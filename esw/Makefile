
WARNINGS=-W -Wall -Wstrict-prototypes -Wmissing-prototypes -Waggregate-return \
    -Wcast-align -Wcast-qual -Wnested-externs -Wshadow -Wbad-function-cast \
    -Wwrite-strings

CFLAGS=-O3 -I.. $(WARNINGS)
CFLAGS+=-ffast-math -fomit-frame-pointer 
#CFLAGS+=-funroll-loops
#CFLAGS+=-march=prescott 
#CFLAGS+= -mtune=native 
# TIP: try adding -openmp or -fopenmp  to enable OPENMP directives and use of multiple cores
#CFLAGS+=-fopenmp
CFLAGS+= $(CFLAGADD)


ifeq "$(NFFT)" ""
 NFFT=1800
endif
ifeq "$(NUMFFTS)" ""
 NUMFFTS=10000
endif

ifeq "$(DATATYPE)" ""
 DATATYPE=float
endif

#BENCHKISS=bm_kiss_$(DATATYPE)
#BENCHFFTW=bm_fftw_$(DATATYPE)
#SELFTEST=st_$(DATATYPE)
#SERVER=tr_$(DATATYPE)
SERVER=server
#TESTKFC=tkfc_$(DATATYPE)
#FASTFILTREAL=ffr_$(DATATYPE)
#SELFTESTSRC=twotonetest.c


#ifeq  "$(DATATYPE)" "int16_t" 
# TYPEFLAGS=-DFIXED_POINT=16
#endif

#ifeq  "$(DATATYPE)" "int32_t"
# TYPEFLAGS=-DFIXED_POINT=32 
#endif

#ifeq  "$(DATATYPE)" "simd"
# TYPEFLAGS=-DUSE_SIMD=1 -msse
#endif


#ifeq  "$(DATATYPE)" "float"
 # fftw needs to be built with --enable-float to build this lib
# FFTWLIB=-lfftw3f
#else
# FFTWLIB=-lfftw3
#endif

#FFTWLIBDIR=-L/usr/local/lib/

SRCFILES=./real_fft_1024.c ./memory_map.c ./fft_peak.c ./red_pitaya_identify.c ./rand.c ./cmd_parse.c

all: $(SERVER) rpdevmem

test: test_parse_command test_frame_layout
	./test_parse_command
	./test_frame_layout

$(SERVER): server.c server.h cmd_parse.h memory_map.h red_pitaya_identify.h fft_peak.h real_fft_1024.h rand.h $(SRCFILES)
	$(CC) -o $@ $(CFLAGS) $+ -lm

rpdevmem: rpdevmem.c
	$(CC) -o $@ $(CFLAGS) rpdevmem.c

test_parse_command: tests/test_parse_command.c cmd_parse.c cmd_parse.h
	$(CC) $(CFLAGS) -I. -I../include -o $@ tests/test_parse_command.c cmd_parse.c -lm

test_frame_layout: tests/test_frame_layout.c memory_map.h fft_peak.h real_fft_1024.h red_pitaya_identify.h
	$(CC) $(CFLAGS) -I. -I.. -o $@ tests/test_frame_layout.c -lm


doc/latex/refman.pdf: Doxyfile
	doxygen Doxyfile
	$(MAKE) -C $(@D)

clean:
	rm -f *~ bm_* st_* tr_* kf_* tkfc_* ff_* ffr_* *.pyc *.pyo *.dat server rpdevmem test_parse_command test_frame_layout
