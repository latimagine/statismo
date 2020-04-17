# [0.12.0](https://github.com/kenavolic/statismo/compare/v0.11.0...v0.12.0)

### Notes

Due to heavy refactoring of the source code, API compatibility with previous versions is broken.

See the [usage section](doc/md/USE.md) to update your code.

The minimum versions for the build toochain (cmake, compilers, platform) and the dependencies are also modified (see [Compatibility](doc/md/INSTALL.md#Compatibility)).

### Breaking Changes

* Modified headers include path
* Modified class hierarchy
* Modified class interface
* Modified class names
* Namespaces creation
* CLI tools options
* ...

### Features

* [+] Logging capabilities
* [+] Thread pool
* [+] Safe containers
* [+] New representers to python wrapper

### Deployment

* [+] Docker file
* [+] Conan package (experimental)
* [-] Debian packaging
* [-] Homebrew packaging

### Bug Fixes

* VTK module python wrapping (source and tests)
* Memory leaks

### Sanity

* Add formatting with clang-format
* Add coding rules enforcement with clang-tidy

### Refactoring

* Update all dependencies version
* Remove boost from dependency
* Use smart memory management
* Use of modern cmake and c++
* Application of DRY principle
* Update header inclusion path with project/module name
* ...

# [0.7.0..0.11.0]

See [original statismo releases](https://github.com/statismo/statismo/releases) for more details.