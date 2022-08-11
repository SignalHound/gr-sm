/* -*- c++ -*- */
/*
 * Copyright 2022 Signal Hound.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "iq_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sm {

using output_type = gr_complex;
iq::sptr iq::make(double center,
                  double reflevel,
                  int decimation,
                  double bandwidth,
                  bool filter,
                  bool purge,
                  bool networked)
{
    return gnuradio::make_block_sptr<iq_impl>(
        center, reflevel, decimation, bandwidth, filter, purge, networked);
}


void ERROR_CHECK(SmStatus status)
{
    if(status != smNoError) {
        bool isWarning = status > smNoError;
        std::cout << "** " << (isWarning ? "Warning: " : "Error: ") << smGetErrorString(status) << " **" << "\n";
        if(!isWarning) abort();
    }
}

iq_impl::iq_impl(double center,
                 double reflevel,
                 int decimation,
                 double bandwidth,
                 bool filter,
                 bool purge,
                 bool networked)
    : gr::sync_block("iq",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 1, sizeof(output_type))),
    _handle(-1),
    _center(center),
    _reflevel(reflevel),
    _decimation(decimation),
    _bandwidth(bandwidth),
    _filter(filter),
    _purge(purge),
    _networked(networked),
    _param_changed(true),
    _buffer(0),
    _len(0)
{
    std::cout << "\nAPI Version: " << smGetAPIVersion() << "\n";

    // Open device
    if(_networked) {
        ERROR_CHECK(smOpenNetworkedDevice(&_handle, SM_ADDR_ANY, SM_DEFAULT_ADDR, SM_DEFAULT_PORT));
    } else {
        ERROR_CHECK(smOpenDevice(&_handle));
    }

    SmDeviceType devType;
    int serial;
    ERROR_CHECK(smGetDeviceInfo(_handle, &devType, &serial));    
    std::cout << "Serial Number: "<< serial << "\n";
}

iq_impl::~iq_impl() 
{
    smAbort(_handle);
    smCloseDevice(_handle);

    if(_buffer) delete [] _buffer;
}

void
iq_impl::set_center(double center) {
    gr::thread::scoped_lock lock(_mutex);
    _center = center;
    _param_changed = true;
}

void
iq_impl::set_reflevel(double reflevel) {
    gr::thread::scoped_lock lock(_mutex);
    _reflevel = reflevel;
    _param_changed = true;
}

void
iq_impl::set_decimation(int decimation) {
    gr::thread::scoped_lock lock(_mutex);
    _decimation = decimation;
    _param_changed = true;
}

void
iq_impl::set_bandwidth(double bandwidth) {
    gr::thread::scoped_lock lock(_mutex);
    _bandwidth = bandwidth;
    _param_changed = true;
}

void
iq_impl::set_filter(bool filter) {
    gr::thread::scoped_lock lock(_mutex);
    _filter = filter;
}

void
iq_impl::set_purge(bool purge) {
    gr::thread::scoped_lock lock(_mutex);
    _purge = purge;
}

void
iq_impl::set_networked(bool networked) {
    gr::thread::scoped_lock lock(_mutex);
    _networked = networked;
}

void
iq_impl::configure() {
    gr::thread::scoped_lock lock(_mutex);

    // Configure
    ERROR_CHECK(smSetIQCenterFreq(_handle, _center));
    ERROR_CHECK(smSetRefLevel(_handle, _reflevel));
    ERROR_CHECK(smSetIQSampleRate(_handle, _decimation));
    ERROR_CHECK(smSetIQBandwidth(_handle, (SmBool)_filter, _bandwidth));
    ERROR_CHECK(smSetIQDataType(_handle, smDataType32fc));

    // Initiate for I/Q streaming
    ERROR_CHECK(smConfigure(_handle, smModeIQ));

    // Get I/Q streaming info
    double sampleRate, actualBandwidth;
    ERROR_CHECK(smGetIQParameters(_handle, &sampleRate, &actualBandwidth));
    std::cout << "\nSample Rate: "<< sampleRate << "\n";
    std::cout << "Actual Bandwidth: "<< actualBandwidth << "\n";
}

int iq_impl::work(int noutput_items,
                  gr_vector_const_void_star& input_items,
                  gr_vector_void_star& output_items)
{
    auto out = static_cast<output_type*>(output_items[0]);

    // Initiate new configuration if necessary
    if(_param_changed) {
        configure();
        _param_changed = false;
    }

    // Allocate memory if necessary
    if(!_buffer || noutput_items != _len) {
        if(_buffer) delete [] _buffer;
        _buffer = new std::complex<float>[noutput_items];
        _len = noutput_items;
    }

    // Get I/Q
    ERROR_CHECK(smGetIQ(_handle, (float *)_buffer, noutput_items, 0, 0, 0, (SmBool)_purge, 0, 0));

    // Move data to output array
    for(int i = 0; i < noutput_items; i++) {
        out[i] =  _buffer[i];
    }

    return noutput_items;
}

} /* namespace sm */
} /* namespace gr */
