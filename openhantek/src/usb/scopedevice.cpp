// SPDX-License-Identifier: GPL-2.0-or-later

#include <QCoreApplication>
#include <QDebug>
#include <QList>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "scopedevice.h"

#include "hantekdso/dsomodel.h"
#include "hantekprotocol/controlStructs.h"
#include "hantekprotocol/controlcode.h"
#include "models/modelDEMO.h"

// Returns a constant QString with a short description of the given error code,
// this description is intended for displaying to the end user and will be
// in the language set by libusb_setlocale().
// Supported languages:
// libusb-1.0.21 (Win):    "en", "nl", "fr", "ru"
// libusb-1.0.22 (Linux):  "en", "nl", "fr", "ru"
// libusb-1.0.23 (MacOSX): "en", "nl", "fr", "ru", "de", "hu"
const QString libUsbErrorString( int error ) { return QString( libusb_strerror( libusb_error( error ) ) ); }


UniqueUSBid ScopeDevice::computeUSBdeviceID( libusb_device *device ) {
    // Returns a 64-bit value that uniquely identifies a device on the bus
    // bus/ports define a constant plug position
    // VID/FW changes with FW upload
    // bpppppppVVVVFFFF
    //             ^^^^-- Firmware version (16bit)(can change after upload of new FW over old FW)
    //         ^^^^------ Vendor ID (16 bit)(changes with FW upload to device w/o FW)
    //  ^^^^^^^---------- USB ports tree (as shown by "lsusb -t"), max 7 ports, port = 1..15, 0 = none
    // ^----------------- USB bus, bus = 1..15

    // Get device descriptor
    struct libusb_device_descriptor descriptor;
    libusb_get_device_descriptor( device, &descriptor );
    // collect values and arrange them
    UniqueUSBid uid = libusb_get_bus_number( device ) & 0x0F; // typically no more than 15 busses
    const int treeSize = 7;                                   // port tree max size is 7
    uint8_t ports[ treeSize ];
    int nPorts = libusb_get_port_numbers( device, ports, sizeof( ports ) );
    for ( int iii = 0; iii < treeSize; ++iii ) {
        uid <<= 4;
        if ( iii < nPorts )
            uid |= ports[ iii ] & 0x0F;
    }
    uid <<= 16;
    uid |= descriptor.idVendor;
    uid <<= 16;
    uid |= descriptor.bcdDevice;
    return uid;
}


ScopeDevice::ScopeDevice( DSOModel *model, libusb_device *device, unsigned findIteration, int verboseLevel )
    : model( model ), device( device ), findIteration( findIteration ), verboseLevel( verboseLevel ),
      uniqueUSBdeviceID( computeUSBdeviceID( device ) ) {
    libusb_ref_device( device );
    libusb_get_device_descriptor( device, &descriptor );
}


ScopeDevice::ScopeDevice( int verboseLevel )
    : model( new ModelDEMO ), device( nullptr ), verboseLevel( verboseLevel ), uniqueUSBdeviceID( 0 ), realHW( false ) {}


bool ScopeDevice::connectDevice( QString &errorMessage ) {
    if ( isDemoDevice() )
        return true;
    if ( !device ) {
        errorMessage = QCoreApplication::translate( "ScopeDevice", "Couldn't open device: device is no longer available" );
        return false;
    }
    if ( needsFirmware() )
        return false;
    if ( isConnected() )
        return true;

    // Open device
    int errorCode = libusb_open( device, &( handle ) );
    if ( errorCode != LIBUSB_SUCCESS ) {
        handle = nullptr;
        errorMessage =
            QCoreApplication::translate( "ScopeDevice", "Couldn't open device: %1" ).arg( libUsbErrorString( errorCode ) );
        return false;
    }
    serialNumber = readUSBdescriptor( handle, descriptor.iSerialNumber );
    // Find and claim interface
    errorCode = LIBUSB_ERROR_NOT_FOUND;
    libusb_config_descriptor *configDescriptor;
    libusb_get_config_descriptor( device, 0, &configDescriptor );
    for ( int interfaceIndex = 0; interfaceIndex < int( configDescriptor->bNumInterfaces ); ++interfaceIndex ) {
        const libusb_interface *pInterface = &configDescriptor->interface[ interfaceIndex ];
        if ( pInterface->num_altsetting < 1 )
            continue;

        const libusb_interface_descriptor *interfaceDescriptor = &pInterface->altsetting[ 0 ];
        if ( interfaceDescriptor->bInterfaceClass == LIBUSB_CLASS_VENDOR_SPEC && interfaceDescriptor->bInterfaceSubClass == 0 &&
             interfaceDescriptor->bInterfaceProtocol == 0 ) {
            errorCode = claimInterface( interfaceDescriptor );
            break;
        }
    }

    libusb_free_config_descriptor( configDescriptor );

    if ( errorCode != LIBUSB_SUCCESS ) {
        errorMessage = QString( "%1 (%2:%3)" )
                           .arg( libUsbErrorString( errorCode ) )
                           .arg( libusb_get_bus_number( device ), 3, 10, QLatin1Char( '0' ) )
                           .arg( libusb_get_device_address( device ), 3, 10, QLatin1Char( '0' ) );
        return false;
    }
    disconnected = false;
    return true;
}


ScopeDevice::~ScopeDevice() {
    disconnectFromDevice( true );
    if ( device != nullptr )
        libusb_unref_device( device );
    device = nullptr;
}


int ScopeDevice::claimInterface( const libusb_interface_descriptor *interfaceDescriptor ) {
    int errorCode = libusb_claim_interface( handle, interfaceDescriptor->bInterfaceNumber );
    if ( errorCode < 0 )
        return errorCode;

    nInterface = interfaceDescriptor->bInterfaceNumber;

    // Check the maximum endpoint packet size
    const libusb_endpoint_descriptor *endpointDescriptor;
    outPacketLength = 0;
    inPacketLength = 0;
    for ( int endpoint = 0; endpoint < interfaceDescriptor->bNumEndpoints; ++endpoint ) {
        endpointDescriptor = &( interfaceDescriptor->endpoint[ endpoint ] );
        if ( endpointDescriptor->bEndpointAddress == HANTEK_EP_OUT ) {
            outPacketLength = endpointDescriptor->wMaxPacketSize;
        } else if ( endpointDescriptor->bEndpointAddress == HANTEK_EP_IN ) {
            inPacketLength = endpointDescriptor->wMaxPacketSize;
        }
    }
    return LIBUSB_SUCCESS;
}


void ScopeDevice::disconnectFromDevice( bool expected ) {
    expectedDisconnect |= expected;
    stopStream(); // before the handle is closed
    if ( disconnected && !handle )
        return;

    disconnected = true;
    if ( !device )
        return;

    if ( handle ) {
        // Release claimed interface
        if ( nInterface != -1 )
            libusb_release_interface( handle, nInterface );
        nInterface = -1;

        // Close device handle
        libusb_close( handle );
    }
    handle = nullptr;

    emit deviceDisconnected( expectedDisconnect );
}


bool ScopeDevice::isConnected() { return isDemoDevice() || ( !disconnected && handle != nullptr ); }


bool ScopeDevice::needsFirmware() {
    return descriptor.idProduct != model->productID || descriptor.idVendor != model->vendorID ||
           descriptor.bcdDevice < model->firmwareVersion;
}


int ScopeDevice::bulkTransfer( unsigned char endpoint, const unsigned char *data, unsigned int length, int attempts,
                               unsigned int timeout ) {
    if ( !handle )
        return LIBUSB_ERROR_NO_DEVICE;

    int errorCode = LIBUSB_ERROR_TIMEOUT;
    int transferred = 0;
    for ( int attempt = 0; ( attempt < attempts || attempts == -1 ) && errorCode == LIBUSB_ERROR_TIMEOUT; ++attempt )
        errorCode =
            libusb_bulk_transfer( handle, endpoint, const_cast< unsigned char * >( data ), int( length ), &transferred, timeout );

    if ( errorCode == LIBUSB_ERROR_NO_DEVICE )
        disconnectFromDevice();
    if ( errorCode < 0 )
        return errorCode;
    else
        return transferred;
}


static const size_t STREAM_RING_BYTES = size_t( 16 ) << 20;
static const int STREAM_TRANSFERS = 16;


bool ScopeDevice::startStream( unsigned chunkBytes ) {
    stopStream();
    std::lock_guard< std::mutex > control( streamControlMutex );
    if ( !handle || !usbContext || disconnected )
        return false;
    chunkBytes = std::max( 1024u, std::min( chunkBytes, 256u * 1024 ) ) / 512 * 512;
    {
        std::lock_guard< std::mutex > lock( streamMutex );
        ring.assign( STREAM_RING_BYTES, 0 );
        ringHead = ringSize = 0;
        streamError = 0;
        ringOverflow = false;
    }
    streamStopping = false;
    streamBuffers.assign( STREAM_TRANSFERS, std::vector< unsigned char >( chunkBytes ) );
    streamTransfers.assign( STREAM_TRANSFERS, nullptr );
    int submitted = 0;
    for ( int iii = 0; iii < STREAM_TRANSFERS; ++iii ) {
        libusb_transfer *t = libusb_alloc_transfer( 0 );
        if ( !t )
            break;
        libusb_fill_bulk_transfer( t, handle, HANTEK_EP_IN, streamBuffers[ size_t( iii ) ].data(), int( chunkBytes ), streamCallback,
                                   this, 0 ); // no timeout
        streamTransfers[ size_t( iii ) ] = t;
        streamOutstanding = submitted + 1; // count before submit, the callback may run immediately
        if ( libusb_submit_transfer( t ) != 0 ) {
            streamOutstanding = submitted;
            libusb_free_transfer( t );
            streamTransfers[ size_t( iii ) ] = nullptr;
            break;
        }
        ++submitted;
    }
    if ( submitted == 0 )
        return false;
    streamThread = std::thread( [ this ]() {
        struct timeval tv = { 0, 100000 };
        while ( streamOutstanding > 0 )
            libusb_handle_events_timeout_completed( usbContext, &tv, nullptr );
    } );
    return true;
}


void ScopeDevice::stopStream() {
    std::lock_guard< std::mutex > control( streamControlMutex );
    if ( !streamThread.joinable() )
        return;
    streamStopping = true;
    for ( libusb_transfer *t : streamTransfers )
        if ( t )
            libusb_cancel_transfer( t ); // callbacks (event thread) will not resubmit
    streamThread.join();
    for ( libusb_transfer *t : streamTransfers )
        if ( t )
            libusb_free_transfer( t );
    streamTransfers.clear();
    streamBuffers.clear();
    {
        std::lock_guard< std::mutex > lock( streamMutex );
        ringHead = ringSize = 0;
    }
    streamCond.notify_all();
}


void ScopeDevice::streamAppend( const unsigned char *data, size_t length ) { // streamMutex is held
    const size_t cap = ring.size();
    if ( length > cap ) {
        data += length - cap;
        length = cap;
        ringOverflow = true;
    }
    if ( ringSize + length > cap ) { // consumer too slow: drop oldest data
        size_t drop = ringSize + length - cap;
        ringHead = ( ringHead + drop ) % cap;
        ringSize -= drop;
        ringOverflow = true;
    }
    size_t tail = ( ringHead + ringSize ) % cap;
    size_t first = std::min( length, cap - tail );
    std::copy( data, data + first, ring.begin() + long( tail ) );
    std::copy( data + first, data + length, ring.begin() );
    ringSize += length;
}


void LIBUSB_CALL ScopeDevice::streamCallback( libusb_transfer *transfer ) {
    ScopeDevice *self = static_cast< ScopeDevice * >( transfer->user_data );
    bool resubmit = !self->streamStopping;
    {
        std::lock_guard< std::mutex > lock( self->streamMutex );
        if ( transfer->actual_length > 0 )
            self->streamAppend( transfer->buffer, size_t( transfer->actual_length ) );
        switch ( transfer->status ) {
        case LIBUSB_TRANSFER_COMPLETED:
        case LIBUSB_TRANSFER_TIMED_OUT:
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            resubmit = false;
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            self->streamError = LIBUSB_ERROR_NO_DEVICE;
            resubmit = false;
            break;
        default:
            self->streamError = LIBUSB_ERROR_IO;
            resubmit = false;
            break;
        }
    }
    if ( resubmit && libusb_submit_transfer( transfer ) == 0 ) {
        self->streamCond.notify_all();
        return;
    }
    --self->streamOutstanding;
    self->streamCond.notify_all();
}


int ScopeDevice::readStream( unsigned char *data, unsigned length, int timeoutMs ) {
    std::unique_lock< std::mutex > lock( streamMutex );
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds( timeoutMs );
    while ( ringSize < length && !streamError && streamOutstanding > 0 ) {
        if ( hasStopped() || std::chrono::steady_clock::now() >= deadline )
            break;
        streamCond.wait_for( lock, std::chrono::milliseconds( 50 ) );
    }
    const size_t n = std::min< size_t >( length, ringSize );
    if ( n == 0 && streamError )
        return streamError;
    const size_t cap = ring.size();
    const size_t first = std::min( n, cap - ringHead );
    std::copy( ring.begin() + long( ringHead ), ring.begin() + long( ringHead + first ), data );
    std::copy( ring.begin(), ring.begin() + long( n - first ), data + first );
    ringHead = ( ringHead + n ) % cap;
    ringSize -= n;
    return int( n );
}


int ScopeDevice::bulkReadMulti( unsigned char *data, unsigned length, bool captureSmallBlocks, unsigned &received, int attempts ) {
    if ( !handle || disconnected )
        return LIBUSB_ERROR_NO_DEVICE;
    int retCode = 0;
    if ( verboseLevel > 6 )
        qDebug() << "      ScopeDevice::bulkReadMulti()" << length;
    if ( captureSmallBlocks ) { // used in roll mode
        // slow data is read in smaller chunks to enable quick screen update
        const unsigned packetLength = 512 * 78; // ~200 samples on screen (200x oversampling)
        retCode = int( packetLength );
        unsigned int packet;
        received = 0;
        for ( packet = 0; received < length && retCode == int( packetLength ); ++packet ) {
            if ( hasStopped() )
                break;
            retCode = bulkTransfer( HANTEK_EP_IN, data + packet * packetLength, qMin( length - unsigned( received ), packetLength ),
                                    attempts, HANTEK_TIMEOUT_MULTI * 10 );
            if ( retCode > 0 )
                received += unsigned( retCode );
            if ( verboseLevel > 6 )
                qDebug() << "      ScopeDevice::bulkReadMulti() bulkTransfer retCode" << retCode;
        }
        if ( verboseLevel > 6 )
            qDebug() << "      scopeDevice::bulkReadMulti() packet, received" << packet << received;
        if ( received > 0 )
            retCode = int( received );
        return retCode;
    } else {
        // more stable if fast data is read as one big block (up to 4 MB)
        if ( hasStopped() )
            return 0;
        retCode = bulkTransfer( HANTEK_EP_IN, data, length, attempts, HANTEK_TIMEOUT_MULTI * length / inPacketLength );
        if ( retCode < 0 )
            received = 0;
        else
            received = unsigned( retCode );
        stopTransfer = false;
        return retCode;
    }
}


// static QString hexString( unsigned char byte ) { return QString( "0x%1" ).arg( byte, 2, 16, QLatin1Char( '0' ) ); }

static QString usbTypeString( int type ) {
    QString t = "";
    switch ( type & LIBUSB_REQUEST_TYPE_RESERVED ) {
    case LIBUSB_REQUEST_TYPE_STANDARD:
        t += "Standard";
        break;
    case LIBUSB_REQUEST_TYPE_CLASS:
        t += "Class";
        break;
    case LIBUSB_REQUEST_TYPE_VENDOR:
        t += "Vendor";
        break;
    case LIBUSB_REQUEST_TYPE_RESERVED:
        t += "Reserved";
        break;
    }
    switch ( type & LIBUSB_ENDPOINT_DIR_MASK ) {
    case LIBUSB_ENDPOINT_OUT:
        t += " Out";
        break;
    case LIBUSB_ENDPOINT_IN:
        t += " In";
        break;
    }
    return t;
}


static QString usbControlCode( uint8_t value ) {
    if ( value >= 0xe0 && value <= 0xe6 )
        return controlNames[ value - 0xe0 ];
    else if ( value == uint8_t( ControlCode::CONTROL_INTERNAL ) )
        return "INTERNAL";
    else if ( value == uint8_t( ControlCode::CONTROL_EEPROM ) )
        return "EEPROM";
    else if ( value == uint8_t( ControlCode::CONTROL_MEMORY ) )
        return "MEMORY";
    else
        return "0x" + QString::number( value, 16 );
}

// max control transfer (write) size is 64 bytes
int ScopeDevice::controlTransfer( unsigned char type, unsigned char request, unsigned char *data, unsigned int length, int value,
                                  int index, int attempts ) {
    if ( !handle || disconnected )
        return LIBUSB_ERROR_NO_DEVICE;

    int errorCode = LIBUSB_ERROR_TIMEOUT;
    for ( int attempt = 0; ( attempt < attempts || attempts == -1 ) && errorCode == LIBUSB_ERROR_TIMEOUT; ++attempt )
        errorCode = libusb_control_transfer( handle, type, request, uint16_t( value ), uint16_t( index ), data, uint16_t( length ),
                                             HANTEK_TIMEOUT );
    if ( verboseLevel > 6 ) {
        QDebug line = qDebug().noquote() << "      ScopeDevice::controlTransfer()" << usbTypeString( type )
                                         << usbControlCode( request );
        line << value << index << '{';
        for ( unsigned iii = 0; iii < length; ++iii ) {
            if ( length > 8 && 0 == iii % 16 )
                line << "\n     ";
            line << QString::number( data[ iii ], 16 );
        }
        line << "} =" << errorCode;
    }

    if ( errorCode == LIBUSB_ERROR_NO_DEVICE )
        disconnectFromDevice();
    return errorCode;
}


QString ScopeDevice::readUSBdescriptor( libusb_device_handle *handle, uint8_t index ) {
    unsigned char string[ 255 ];
    int ret = libusb_get_string_descriptor_ascii( handle, index, string, sizeof( string ) );
    if ( ret > 0 )
        return QString::fromLatin1( reinterpret_cast< char * >( string ), ret ).trimmed();
    else
        return QString();
}
