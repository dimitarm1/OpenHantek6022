// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "hantekdsocontrol.h"

class CapturingThread : public QThread {
    Q_OBJECT

  public:
    CapturingThread( HantekDsoControl *hdc );
    void quitCapturing() { hdc->capturing = false; }

  private:
    void run() override;
    void capture();
    unsigned getRealSamples( ScopeDevice *device );
    unsigned getDemoSamples();
    unsigned getStreamSamples( ScopeDevice *device );
    void stopStreamIfRunning();
    void xferSamples();
    HantekDsoControl *hdc;
    unsigned channels = 0;
    double effectiveSamplerate = 0;
    bool realSlow = false;
    double samplerate = 0;
    unsigned oversampling = 0;
    unsigned rawSamplesize = 0;
    unsigned received = 0;
    unsigned gainValue[ 2 ] = { 0, 0 }; // 1,2,5,10,..
    unsigned gainIndex[ 2 ] = { 0, 0 }; // index 0..7
    unsigned tag = 0;
    bool valid = true;
    bool freeRun = false;
    bool blockContinued = false; ///< this block directly continues the previous one
    std::vector< unsigned char > data;
    std::vector< unsigned char > *dp = &data;
    std::vector< unsigned char > carry; ///< tail of the previous block, overlaps the skipped lead-in of the next block
};
