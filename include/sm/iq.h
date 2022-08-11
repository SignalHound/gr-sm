/* -*- c++ -*- */
/*
 * Copyright 2022 Signal Hound.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SM_IQ_H
#define INCLUDED_SM_IQ_H

#include <gnuradio/sync_block.h>
#include <sm/api.h>

namespace gr {
namespace sm {

/*!
 * \brief This block acquires I/Q data from the Signal Hound SM200/435 spectrum analyzer.
 * \ingroup sm
 *
 */
class SM_API iq : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<iq> sptr;

    static sptr make(double center = 1e9,
                     double reflevel = -20,
                     int decimation = 2,
                     double bandwidth = 20e6,
                     bool filter = true,
                     bool purge = false,
                     bool networked = false);

    virtual void set_center(double center) = 0;
    virtual void set_reflevel(double reflevel) = 0;
    virtual void set_decimation(int decimation) = 0;
    virtual void set_bandwidth(double bandwidth) = 0;
    virtual void set_filter(bool filter) = 0;
    virtual void set_purge(bool purge) = 0;
    virtual void set_networked(bool networked) = 0;
};

} // namespace sm
} // namespace gr

#endif /* INCLUDED_SM_IQ_H */
