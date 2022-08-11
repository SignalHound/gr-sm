/* -*- c++ -*- */
/*
 * Copyright 2022 Signal Hound.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SM_IQ_IMPL_H
#define INCLUDED_SM_IQ_IMPL_H

#include <sm/iq.h>
#include <sm/sm_api.h>

namespace gr {
namespace sm {

class iq_impl : public iq
{
private:
      int _handle;

      double _center;
      double _reflevel;
      int _decimation;
      double _bandwidth;
      bool _filter;
      bool _purge;
      bool _networked;

      gr::thread::mutex _mutex;
      bool _param_changed;

      std::complex<float> *_buffer;
      int _len;

public:
    iq_impl(double center,
            double reflevel,
            int decimation,
            double bandwidth,
            bool filter,
            bool purge,
            bool networked);
    ~iq_impl();

    void set_center(double center);
    void set_reflevel(double reflevel);
    void set_decimation(int decimation);
    void set_bandwidth(double bandwidth);
    void set_filter(bool filter);
    void set_purge(bool purge);
    void set_networked(bool networked);

    void configure();

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace sm
} // namespace gr

#endif /* INCLUDED_SM_IQ_IMPL_H */
