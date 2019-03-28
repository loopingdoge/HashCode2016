# HashCode2016
Artificial Intelligence course project based on the [HashCode 2016 problem](https://hashcode.withgoogle.com/2016/tasks/hashcode2016_qualification_task.pdf)

## Requirements
- [**Jinnja2**](http://jinja.pocoo.org/)
  - `pip install Jinja2` (may require sudo)

### If you want to use Docker
Make sure you have already installed both [Docker Engine](https://docs.docker.com/install/) and [Docker Compose](https://docs.docker.com/compose/install/). After that:

1. Create the image (about 600MB):
    ```bash
    docker build --tag=hashcode2016 .
    ```

2. Run the service detached:
    ```bash
    docker compose up -d
    docker ps # check the container status
    ```

3. Attach the container with a shell and test the solver:
    ```bash
    docker ps # look for the cointainer name
    docker exec -it hashcode2016_hc_1 /bin/bash
    cd project
    python src/execute.py test
    exit # stop the container, otherwise use Ctrl-p Ctrl-q
    ```

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