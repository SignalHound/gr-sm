/* -*- c++ -*- */

#define SM200A_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sm200a_swig_doc.i"

%{
#include "sm200a/iq.h"
%}


%include "sm200a/iq.h"
GR_SWIG_BLOCK_MAGIC2(sm200a, iq);
