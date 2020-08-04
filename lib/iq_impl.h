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

#ifndef INCLUDED_SM200_IQ_IMPL_H
#define INCLUDED_SM200_IQ_IMPL_H

#include <sm200/iq.h>
#include <sm200/sm_api.h>

namespace gr {
  namespace sm200 {

    class iq_impl : public iq
    {
    private:
      int d_handle;

      double d_center;
      int d_decimation;
      double d_bandwidth;
      bool d_filter;
      bool d_purge;
      bool d_networked;

      gr::thread::mutex d_mutex;
      bool d_param_changed;

      std::complex<float> *d_buffer;
      bool d_len;

    public:
      iq_impl(double center,
              int decimation,
              double bandwidth,
              bool filter,
              bool purge,
              bool networked);
      ~iq_impl();

      void set_center(double center);
      void set_decimation(int decimation);
      void set_bandwidth(double bandwidth);
      void set_filter(bool filter);
      void set_purge(bool purge);
      void set_networked(bool networked);

      void configure();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace sm200
} // namespace gr

#endif /* INCLUDED_SM200_IQ_IMPL_H */

