/* -*- c++ -*- */

#define SM200_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sm200_swig_doc.i"

%{
#include "sm200/iq.h"
%}


%include "sm200/iq.h"
GR_SWIG_BLOCK_MAGIC2(sm200, iq);
