# colav-unsafe-set

[![PyPI - Version](https://img.shields.io/pypi/v/colav-unsafe-set.svg)](https://pypi.org/project/colav-unsafe-set)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/colav-unsafe-set.svg)](https://pypi.org/project/colav-unsafe-set)
<!--[![PyPI - Protobuf Version]()]-->
This package contains implementation of the custom risk assesment method for motion planners called unsafe set as defined in paper [geometric motion planning in dynamic environments](). 
The following is the high level equation which this package implements.

![image](./docs/unsafe_set_calculation.png)

Show image of geometric unsafe set: 
![image](./docs/unsafe_set_diagram.png)

-----

## Table of Contents

- [Installation](#installation)
- [Structure](#structure)
- [Usage](#usage)
- [License](#license)

## Installation

```bash
pip install colav-unsafe-set
```

## Structure
The src code in [colav_protobuf](https://github.com/RyanMcKeeQUB/colav_protobuf) shows that the project is organised into main directories: 
- [Tests](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/tests): The tests directory contains a variety of unit tests ensuring that the pkg is working as expected and are called as apart of the CI/CD pipeline defined by the [github_action](./.github/workflows/workflow.yml)
- [src](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/src/): The src contains three pkgs
    -   [colav_proto](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/src/colav_proto/)the original proto files which were compiled by protobuf compiler [protos](./src/colav_proto/) these are un-importable but will give you a good idea of the proto structure.
    - [colav_protobuf](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/src/colav_protobuf/): This is the importable python pkg you can import the different messages types as follows: 
    - [examples](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/src/colav_protobuf/examples/): This pkg contains mock of each of the protobufs for example and testing purposes.

## Usage
When pkg is installed, Using it is simple imports are as follows. 

```python
```

Examples of these object initiations are shown in the [examples](https://github.com/RyanMcKeeQUB/colav_protobuf/tree/main/src/colav_protobuf/examples/) which contains a number of importable message exmaples.
how to publish it via a python socket: 

Here is a sample of proto creation of a agent configuration message: and 

```python
```


## License

`colav-protobuf` is distributed under the terms of the [MIT](https://github.com/RyanMcKeeQUB/colav-unsafe-set/tree/main/LICENSE) license.
