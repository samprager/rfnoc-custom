/* -*- c++ -*- */

#define WAVEGEN_API
#define ETTUS_API

%include "gnuradio.i"/*			*/// the common stuff

//load generated python docstrings
%include "wavegen_swig_doc.i"
//Header from gr-ettus
%include "ettus/device3.h"
%include "ettus/rfnoc_block.h"
%include "ettus/rfnoc_block_impl.h"

%{
#include "ettus/device3.h"
#include "ettus/rfnoc_block_impl.h"
#include "wavegen/wavegen.h"
%}

%include "wavegen/wavegen.h"
GR_SWIG_BLOCK_MAGIC2(wavegen, wavegen);
