%module swig_parser

%{
#include "swig_parser.h"
%}

%include "std_vector.i"
%include "stdint.i"

%template(PacketVector) std::vector<Packet>;
%template(DataVector) std::vector<uint8_t>;

%include "swig_parser.h"

