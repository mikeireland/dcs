# Commander

Commander is a command line tool interface library, based on the Pyxis project's
commander written by Julien Tom Bernard, which in turn was based on earlier ZMQ
work by Mike Ireland, and inspired by SUSI and CHARA code. The intent in this implementation
is to limit dependencies and maximise simplicity. 

## Quick start

Let's start by creating command bindings for an extremely simple function, which
adds two numbers and returns their result:

```c++
int add(int i, int j) {
    return i + j;
}
```

For simplicity we'll put both this function and the binding code into
a file named `example.cpp` with the following contents:

```cpp
#include <commander/commander.h>

int add(int i, int j) {
    return i + j;
}

COMMANDER_REGISTER(m) {
    m.def("add", &add, "A function that adds two numbers");
}
```

Now, all you need is to compile the source file and run the executable with for instance the
`commander::Server`:

```cpp

int main(int argc, char* argv[]) {
    commander::Server(argc, argv).run();
}
```

and use it as follow:

```bash
./example --command add [2,3]
5
```

## Installation

### Dependencies and Deployment:

`commander` requires only a C++ compiler. To install the examples in the build directory,
you just need:

```bash
make
```

However, if you don't want to hack the Makefile for a new system, then you also need 
autoconf and autotools. In this case, both making in the build directory and installing
is:

```bash
./configure
make
sudo make install
```

## Build documentation

To build the documentation, you need to install `doxygen`, `breathe`, `Sphinx`, `sphinx-rtd-theme`, `sphinx-rtd-dark-mode`.

To install doxygen, refer to your OS package manager. For the rest, run:

```bash
pip install -r requirements.txt
```

To build the documentation, run:

```bash
cd docs
make html
```

It will produce the documentation in `build/docs/sphinx/html`.
