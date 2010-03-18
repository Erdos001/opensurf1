# Indicates compilator to use
CC      = g++

# Specifies compilator options
CFLAGS  = -O3 -Wall `pkg-config --cflags opencv` -D LINUX
LDFLAGS = 
LDLIBS  = `pkg-config --libs opencv`

# Files extensions .cpp, .o
SUFFIXES = .cpp .o 
.SUFFIXES: $(SUFFIXES) .

# Name of the main program
PROG  = surf

# Object files .o necessary to build the main program
OBJS  = fasthessian.o integral.o main.o surf.o utils.o ipoint.o
 
all: $(PROG)

# Compilation and link
$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $(PROG) $(OBJS) $(LDLIBS)

.cpp.o:
	$(CC)   $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(PROG)
	-rm -f *.o
