/* -*- c++ -*- */
/*
 * Copyright (C) 2018 Signal Hound, Inc. <support@signalhound.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "iq_impl.h"

namespace gr {
    namespace sm200 {

        iq::sptr
        iq::make(double center,
                 int decimation,
                 double bandwidth,
                 bool filter,
                 bool purge,
                 bool networked)
        {
            return gnuradio::get_initial_sptr(
                new iq_impl(center, decimation, bandwidth, filter, purge, networked)
            );
        }

        void ERROR_CHECK(SmStatus status) {
            if(status != smNoError) {
                bool isWarning = status < smNoError ? false : true;
                std::cout << "** " << (isWarning ? "Warning: " : "Error: ") << smGetErrorString(status) << " **" << "\n";
                if(!isWarning) abort();
            }
        }

        iq_impl::iq_impl(double center,
                         int decimation,
                         double bandwidth,
                         bool filter,
                         bool purge,
                         bool networked) :
            gr::sync_block("iq",
                           gr::io_signature::make(0, 0, 0),
                           gr::io_signature::make(1, 1, sizeof(gr_complex))),
            d_handle(-1),
            d_center(center),
            d_decimation(decimation),
            d_bandwidth(bandwidth),
            d_filter(filter),
            d_purge(purge),
            d_networked(networked),
            d_param_changed(true),
            d_buffer(0),
            d_len(0)
        {
            std::cout << "\nAPI Version: " << smGetAPIVersion() << "\n";

            // Open device
            if(d_networked) {
                ERROR_CHECK(smOpenNetworkedDevice(&d_handle, SM200_ADDR_ANY, SM200_DEFAULT_ADDR, SM200_DEFAULT_PORT));
            } else {
                ERROR_CHECK(smOpenDevice(&d_handle));
            }

            int serial;
            SmDeviceType deviceType;
            ERROR_CHECK(smGetDeviceInfo(d_handle, &deviceType, &serial));
            std::cout << "Serial Number: "<< serial << "\n";
        }

        iq_impl::~iq_impl()
        {
            smAbort(d_handle);
            smCloseDevice(d_handle);

            if(d_buffer) delete [] d_buffer;
        }

        void
        iq_impl::set_center(double center) {
            gr::thread::scoped_lock lock(d_mutex);
            d_center = center;
            d_param_changed = true;
        }

        void
        iq_impl::set_decimation(int decimation) {
            gr::thread::scoped_lock lock(d_mutex);
            d_decimation = decimation;
            d_param_changed = true;
        }

        void
        iq_impl::set_bandwidth(double bandwidth) {
            gr::thread::scoped_lock lock(d_mutex);
            d_bandwidth = bandwidth;
            d_param_changed = true;
        }

        void
        iq_impl::set_filter(bool filter) {
            gr::thread::scoped_lock lock(d_mutex);
            d_filter = filter;
        }

        void
        iq_impl::set_purge(bool purge) {
            gr::thread::scoped_lock lock(d_mutex);
            d_purge = purge;
        }

        void
        iq_impl::set_networked(bool networked) {
            gr::thread::scoped_lock lock(d_mutex);
            d_networked = networked;
        }

        void
        iq_impl::configure() {
            gr::thread::scoped_lock lock(d_mutex);

            // Configure
            ERROR_CHECK(smSetIQCenterFreq(d_handle, d_center));
            ERROR_CHECK(smSetIQSampleRate(d_handle, d_decimation));
            ERROR_CHECK(smSetIQBandwidth(d_handle, (SmBool)d_filter, d_bandwidth));
            ERROR_CHECK(smSetIQDataType(d_handle, smDataType32fc));

            // Initiate for IQ streaming
            ERROR_CHECK(smConfigure(d_handle, smModeIQ));

            // Get IQ streaming info
            double actualBandwidth, sampleRate;
            ERROR_CHECK(smGetIQParameters(d_handle, &sampleRate, &actualBandwidth));
            std::cout << "\nActual Bandwidth: "<< actualBandwidth << "\n";
            std::cout << "Sample Rate: "<< sampleRate << "\n";
        }

        int
        iq_impl::work(int noutput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
        {
            std::complex<float> *o = (std::complex<float> *)output_items[0];

            // Initiate new configuration if necessary
            if(d_param_changed) {
                configure();
                d_param_changed = false;
            }

            // Allocate memory if necessary
            if(!d_buffer || noutput_items != d_len) {
                if(d_buffer) delete [] d_buffer;
                d_buffer = new std::complex<float>[noutput_items];
                d_len = noutput_items;
            }

            // Get IQ
            ERROR_CHECK(smGetIQ(d_handle, (float *)d_buffer, noutput_items, 0, 0, 0, (SmBool)d_purge, 0, 0));

            // Move data to output array
            for(int i = 0; i < noutput_items; i++) {
                o[i] =  d_buffer[i];
            }

            return noutput_items;
        }

    } /* namespace sm200 */
} /* namespace gr */

