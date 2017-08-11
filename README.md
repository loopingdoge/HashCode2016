# HashCode2016
Artificial Intelligence course project based on the [HashCode 2016 problem](https://hashcode.withgoogle.com/2016/tasks/hashcode2016_qualification_task.pdf)

## Requirements
- [**Jinnja2**](http://jinja.pocoo.org/)
  - `pip install Jinja2` (may require sudo)

## Input Generator

### Usage
Requires the args:
- map rows
- map cols
- drones number
- max turns
- drones payload
- products number
- warehouses number
- orders number


After the execution it will output a file in `./in/generated.in`.
#### Example
```
$ python src/generate_input.py 50 50 10 500 250 5 3 3
```