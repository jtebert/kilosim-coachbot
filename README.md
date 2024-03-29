# Kilosim Coachbot

A Kilosim version of the Coachbot (for PIswarm)

## Download/install

```shell
git clone --recurse-submodules https://github.com/jtebert/kilosim-coachbot
```

This will clone the current repository as well as the `kilosim` dependency.

If you clone normally and forget to do it with the submodules, you can clone them in later:

```shell
git submodule update --init --recursive
```

## Build

The primary purpose of this is to be a library of the Coachbot that can be used for PIswarm simulations. But I'm also including an executable option for debugging.

```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..
make # Compile the Coachbot library
make coachbot_example # OPTIONAL: Compile the executable example code
make install # Install the library (and example, if you compiled it)
```

## References

[Kilosim](https://github.com/jtebert/kilosim)

[Example Kilosim project (demo)](https://github.com/jtebert/kilosim-demo)