// SPDX-License-Identifier: GPL-2.0-or-later

// #define TIMESTAMPDEBUG

#include "capturing.h"
#include "usb/scopedevice.h"
#include <QDebug>
#include <cmath>


/// Up to this hardware samplerate the scope is read as one gapless stream (see getStreamSamples()).
/// Low effective rates use a high hardware rate with oversampling (e.g. 10 MS/s for 1 MS/s), 16 MS/s is tested.
static const double STREAM_MAX_SAMPLERATE = 16e6;


CapturingThread::CapturingThread( HantekDsoControl *hdc ) : hdc( hdc ) {
    if ( hdc->verboseLevel > 1 ) {
        qDebug() << " CapturingThread::CapturingThread()";
        if ( hdc->verboseLevel > 2 )
            qDebug() << "  capturingThread ID: " << currentThreadId();
    }
    hdc->capturing = true;
}


void CapturingThread::run() {
    forever {
        if ( !hdc->capturing || QThread::currentThread()->isInterruptionRequested() ) {
            stopStreamIfRunning();
            hdc->quitSampling();     // stop the scope
            hdc->stopStateMachine(); // stop the state machine
            return;                  // stop this thread
        }
        if ( hdc->scope ) { // device is initialized
            if ( hdc->samplingUI ) {
                capture();
                // add user defined hold-off time to lower CPU load
                QThread::msleep( unsigned( 1000 * hdc->scope->horizontal.acquireInterval ) );
            } else {
                stopStreamIfRunning();
                QThread::msleep( unsigned( hdc->displayInterval ) ); // run slowly
            }
        }
    }
}


static double id2sr( uint8_t timediv ) {
    if ( timediv < 100 ) {   // 1, 2, ..., 48 MS/s
        if ( 11 == timediv ) // fix brain dead coding of sigrok DDS120 FW
            timediv = 10;    // 11 means 10 MS/s (see modelDDS120.cpp)
        return timediv * 1e6;
    } // 1NN -> NN0 kS/s (102 -> 20 kS/s, ..., 150 -> 500 kS/S)
    return ( timediv - 100 ) * 1e4;
}


void CapturingThread::xferSamples() {
    QWriteLocker locker( &hdc->raw.lock );
    if ( !freeRun )
        swap( data, hdc->raw.data );
    hdc->raw.channels = channels;
    hdc->raw.samplerate = samplerate;
    hdc->raw.oversampling = oversampling;
    hdc->raw.gainValue[ 0 ] = gainValue[ 0 ];
    hdc->raw.gainValue[ 1 ] = gainValue[ 1 ];
    hdc->raw.gainIndex[ 0 ] = gainIndex[ 0 ];
    hdc->raw.gainIndex[ 1 ] = gainIndex[ 1 ];
    hdc->raw.freeRun = freeRun;
    hdc->raw.valid = valid;
    hdc->raw.tag = tag;
    hdc->raw.continued = blockContinued;
    if ( valid && !freeRun )
        hdc->recordRawFrame();
}


void CapturingThread::capture() {
    if ( !hdc->samplingStarted )
        return;
    QReadLocker deviceLocker( &hdc->scopeDeviceLock );
    ScopeDevice *device = hdc->scopeDevice;
    if ( !device || hdc->deviceConnectionState != HantekDsoControl::DeviceConnectionState::Connected || !device->isConnected() )
        return;

    int errorCode;
    // Changed settings interrupt the continuous stream, it is restarted below
    if ( device->isStreaming() ) {
        for ( ControlCommand *cc = hdc->firstControlCommand; cc; cc = cc->next ) {
            if ( cc->pending ) {
                device->stopStream();
                break;
            }
        }
    }
    // Send all pending control commands
    ControlCommand *controlCommand = hdc->firstControlCommand;
    while ( controlCommand ) {
        if ( controlCommand->pending ) {
            switch ( int( controlCommand->code ) ) {
            case uint8_t( Hantek::ControlCode::CONTROL_SETGAIN_CH1 ):
                gainValue[ 0 ] = controlCommand->data()[ 0 ];
                gainIndex[ 0 ] = controlCommand->data()[ 1 ];
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETGAIN_CH2 ):
                gainValue[ 1 ] = controlCommand->data()[ 0 ];
                gainIndex[ 1 ] = controlCommand->data()[ 1 ];
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETNUMCHANNELS ):
                if ( realSlow ) { // force 2 channels for slow samplings where roll mode is possible
                    if ( *controlCommand->data() == channels )
                        controlCommand->pending = false;
                    else
                        *controlCommand->data() = 2;
                }
                channels = *controlCommand->data();
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETSAMPLERATE ):
                samplerate = id2sr( controlCommand->data()[ 0 ] );
                uint8_t sampleIndex = controlCommand->data()[ 1 ];
                oversampling = uint8_t( hdc->specification->fixedSampleRates[ sampleIndex ].oversampling );
                effectiveSamplerate = hdc->specification->fixedSampleRates[ sampleIndex ].samplerate;
                if ( !realSlow && effectiveSamplerate < 10e3 &&
                     hdc->scope->trigger.mode == Dso::TriggerMode::ROLL ) { // switch to real slow rolling
                    for ( auto it = data.begin(); it != data.end(); ) {
                        *it++ = hdc->channelOffset[ 0 ]; // fill ch0 with "zeros"
                        *it++ = hdc->channelOffset[ 1 ]; // fill ch1 with "zeros"
                    }
                    QWriteLocker locker( &hdc->raw.lock );
                    hdc->raw.rollMode = false;
                    swap( data, hdc->raw.data ); // "clear screen"
                }
                realSlow = effectiveSamplerate < 10e3;
                if ( realSlow ) {        // roll mode possible?
                    if ( channels != 2 ) // always switch to two channels
                        hdc->modifyCommand< Hantek::ControlSetNumChannels >( Hantek::ControlCode::CONTROL_SETNUMCHANNELS )->setNumChannels( 2 );
                } else {
                    if ( channels == 2 && hdc->isSingleChannel() ) // switch back to real user setting
                        hdc->modifyCommand< Hantek::ControlSetNumChannels >( Hantek::ControlCode::CONTROL_SETNUMCHANNELS )->setNumChannels( 1 );
                }
                break;
            }
            QString name = "";
            if ( controlCommand->code >= 0xe0 && controlCommand->code <= 0xe6 )
                name = Hantek::controlNames[ controlCommand->code - 0xe0 ];
            timestampDebug( QString( "Sending control command 0x%1 (%2): %3" )
                                .arg( QString::number( controlCommand->code, 16 ), name,
                                      hexdecDump( controlCommand->data(), unsigned( controlCommand->size() ) ) ) );
            if ( device->isRealHW() ) { // do the USB communication with scope HW
                errorCode = device->controlWrite( controlCommand );
                if ( errorCode < 0 ) {
                    qWarning( "Sending control command %2x failed: %s", uint8_t( controlCommand->code ),
                              libUsbErrorString( errorCode ).toLocal8Bit().data() );

                    if ( errorCode == LIBUSB_ERROR_NO_DEVICE ) {
                        return;
                    }
                } else {
                    controlCommand->pending = false;
                }
            } else {
                controlCommand->pending = false;
            }
        }
        controlCommand = controlCommand->next;
    }
    valid = true;
    freeRun = hdc->triggerModeNONE() && realSlow;
    // sample step by step into the target if rollMode, else buffer and switch one big block
    dp = freeRun ? &hdc->raw.data : &data;
    rawSamplesize = hdc->grossSampleCount( hdc->getSamplesize() * oversampling ) * channels;
    dp->resize( rawSamplesize, 0x80 );
    if ( tag && freeRun ) // in free run mode transfer settings immediately
        xferSamples();
    if ( 0 == ++tag )
        ++tag; // skip tag==0
    blockContinued = false;
    if ( device->isRealHW() ) {
        if ( !realSlow && !freeRun && samplerate <= STREAM_MAX_SAMPLERATE )
            received = getStreamSamples( device );
        else {
            if ( device->isStreaming() )
                device->stopStream();
            received = getRealSamples( device );
        }
    } else {
        received = getDemoSamples();
    }
    if ( received != rawSamplesize ) {
        // qDebug() << "retval != rawSamplesize" << received << rawSamplesize;
        auto end = dp->end();
        for ( auto it = dp->begin(); it != end; ) {
            *it++ = hdc->channelOffset[ 0 ]; // fill ch0 with "zeros"
            *it++ = hdc->channelOffset[ 1 ]; // fill ch1 with "zeros"
        }
        valid = false;
        hdc->raw.rollMode = false;
    } else {
        hdc->raw.rollMode = true; // one complete buffer available, start to roll
    }
    if ( !freeRun ) // in normal capturing mode transfer after capturing one block
        xferSamples();
}


void CapturingThread::stopStreamIfRunning() {
    QReadLocker deviceLocker( &hdc->scopeDeviceLock );
    if ( hdc->scopeDevice && hdc->scopeDevice->isStreaming() )
        hdc->scopeDevice->stopStream();
    carry.clear();
}


/// Read the next block from the continuous stream instead of re-arming the scope for every block.
/// The first samples of each block are skipped by the display (unstable start after re-arming),
/// so each new block begins with a copy of the end of the previous block. Consecutive displayed blocks
/// then follow each other without any hole in time.
unsigned CapturingThread::getStreamSamples( ScopeDevice *device ) {
    const unsigned samplesPerChannel = rawSamplesize / channels;
    const unsigned skipBytes = ( samplesPerChannel - hdc->netSampleCount( samplesPerChannel ) ) * channels;
    if ( !device->isStreaming() ) {
        // transfers of about 10 ms length
        const unsigned chunk = unsigned( samplerate * channels / 100 );
        if ( !device->startStream( chunk ) )
            return 0;
        int errorCode = device->controlWrite( hdc->getCommand( Hantek::ControlCode::CONTROL_STARTSAMPLING ) );
        if ( errorCode < 0 ) {
            qWarning() << "controlWrite: Starting stream failed: " << libUsbErrorString( errorCode );
            device->stopStream();
            return 0;
        }
        carry.clear();
    }
    unsigned lead = 0;
    if ( skipBytes && skipBytes < rawSamplesize && carry.size() >= skipBytes ) {
        lead = skipBytes;
        std::copy( carry.end() - lead, carry.end(), dp->begin() );
    }
    const unsigned wanted = rawSamplesize - lead;
    int got = device->readStream( dp->data() + lead, wanted, 2000 );
    if ( got < 0 || unsigned( got ) != wanted ) { // timeout, stop or error: restart the stream with the next block
        device->stopStream();
        carry.clear();
        return got < 0 ? 0 : lead + unsigned( got );
    }
    blockContinued = lead > 0;
    carry.assign( dp->end() - skipBytes, dp->end() );
    hdc->raw.received = rawSamplesize;
    return rawSamplesize;
}


unsigned CapturingThread::getRealSamples( ScopeDevice *device ) {
    int errorCode;
    errorCode = device->controlWrite( hdc->getCommand( Hantek::ControlCode::CONTROL_STARTSAMPLING ) );
    if ( errorCode < 0 ) {
        qWarning() << "controlWrite: Getting sample data failed: " << libUsbErrorString( errorCode );
        dp->clear();
        return 0;
    }
    // Save raw data to temporary buffer
    // timestampDebug( QString( "Request packet %1: %2 bytes" ).arg( tag ).arg( rawSamplesize ) );
    hdc->raw.received = 0;
    int retval = device->bulkReadMulti( dp->data(), rawSamplesize, realSlow, hdc->raw.received );
    if ( retval < 0 ) {
        if ( retval == LIBUSB_ERROR_NO_DEVICE )
            device->disconnectFromDevice();
        qWarning() << "bulkReadMulti: Getting sample data failed: " << libUsbErrorString( retval );
        dp->clear();
        return 0;
    }
    // timestampDebug( QString( "Received packet %1: %2 bytes" ).arg( tag ).arg( retval ) );
    return unsigned( retval );
}


unsigned CapturingThread::getDemoSamples() {
    const uint8_t binaryOffset = 0x80; // ADC format: binary offset
    const int8_t V_zero = 0;           // ADC = 0V
    const int8_t V_plus_1 = 25;        // ADC = 1V
    const int8_t V_plus_2 = 50;        // ADC = 2V
    const int8_t V_minus_1 = -25;      // ADC = -1V
    const int8_t V_minus_2 = -50;      // ADC = -2V
    const int gain1 = int( gainValue[ 0 ] );
    const int gain2 = int( gainValue[ 1 ] );
    static int ch1 = 0;
    static int ch2 = 0;
    static int counter = 0;
    unsigned received = 0;
    hdc->raw.received = 0;
    // timestampDebug( QString( "Request dummy packet %1: %2 bytes" ).arg( tag ).arg( rawSamplesize ) );
    int deltaT = 99;
    // deltaT (=99) defines the frequency of the dummy signals:
    // ch1 = 1 kHz and ch2 = 500 Hz
    // uncomment the next two lines to disable sample slowdown for low sample rates
    // if ( samplerate < 10e6 )
    //    deltaT = int( round( deltaT * samplerate / 10e6 ) );
    // adapt demo samples for high sample rates >10 MS/s
    if ( samplerate > 10e6 )
        deltaT = int( round( deltaT * samplerate / 10e6 ) );
    const unsigned packetLength = 512 * 78; // 50 blocks for one screen width of 20000
    unsigned block = 0;
    dp->resize( rawSamplesize, binaryOffset );
    auto end = dp->end();
    // unsigned packet = 0;
    // bool couplingAC1 = hdc->scope->coupling( 0, hdc->specification ) == Dso::Coupling::AC; // not yet used
    bool couplingAC2 = hdc->scope->coupling( 1, hdc->specification ) == Dso::Coupling::AC;
    for ( auto it = dp->begin(); it != end; ++it ) {
        if ( ++counter >= deltaT ) {
            counter = 0;
            if ( --ch1 < V_minus_2 ) {
                ch1 = V_plus_2;
                if ( couplingAC2 )
                    ch2 = ch2 <= V_zero ? V_plus_1 : V_minus_1; // -1V <-> +1V
                else
                    ch2 = ch2 <= V_plus_1 ? V_plus_2 : V_zero; // 0V <-> 2V
            }
        }
        *it = uint8_t( qBound( 0, ch1 * gain1 + binaryOffset, 0xFF ) ); // clip if outside 8bit range
        ++received;
        if ( 2 == channels ) {
            *++it = uint8_t( qBound( 0, ch2 * gain2 + binaryOffset, 0xFF ) ); // clip ..
            ++received;
        }
        if ( ( block += channels ) >= packetLength ) {
            // ++packet;
            block = 0;
            hdc->raw.received = received;
            QThread::usleep( unsigned( 1e6 * packetLength / channels / samplerate ) );
            if ( !hdc->capturing || !hdc->isDeviceAvailable() )
                break;
            QReadLocker deviceLocker( &hdc->scopeDeviceLock );
            if ( hdc->scopeDevice && hdc->scopeDevice->hasStopped() )
                break;
        }
    }
    // timestampDebug( QString( "Received dummy packet %1: %2 bytes" ).arg( packet ).arg( rawSamplesize ) );
    return received;
}
