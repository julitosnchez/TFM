CXX = g++
WARN = -Wall
CFLAGS = $(WARN)
LDFLAGS = $(WARN) -lecf

LIB=$(PCLLIBRARY)
INC=$(PCLINCLUDE)

OBJS = FunctionMinEvalOp.o \
       main.o

BINFILE = funcmin

.PHONY = all clean

all: $(BINFILE)

$(BINFILE): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $(BINFILE) $(LIB) $(INC)

%.o: %.cpp FunctionMinEvalOp.h
	$(CXX) $(CFLAGS) $< -c -o $@

clean:
	rm -f *~ $(OBJS) $(BINFILE)
