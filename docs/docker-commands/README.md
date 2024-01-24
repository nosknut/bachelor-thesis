# Docker Commands

## Docker Compose

### Show running services

```bash
docker-compose ps
```

## Show all services

```bash
docker-compose config --services
```

### Start service

```bash
docker-compose up service
```

### Stop docker-compose service

```bash
docker-compose down service
```

### Start all services

```bash
docker-compose up
```

### Stop all services

```bash
docker-compose down
```

### Start services in detached mode

Starts a docker-compose service in the background so that the terminal is not blocked.
This command can be used to start multiple services.

```bash
docker-compose up -d service1 service2 service3
```

### Open terminal in running docker-compose service

```bash
docker-compose exec service bash
```

### Run docker-compose service with a command

```bash
docker-compose run service command
```

### Run docker-compose service that does not have a command

For containers that have no startup command, this will open the terminal inside the container.

```bash
docker-compose run service
```

## Docker

### Show all docker images

```bash
docker images
```

### Open terminal in running docker-compose service

```bash
docker exec -it service /bin/bash
```

### Delete docker imagges matching pattern

```bash
docker rmi $(docker images -a | grep "pattern" | awk '{print $3}')
```

### List all docker containers including inactive
```bash
docker ps -a
```

### List all docker container ids
```bash
docker ps -aq
```

### Remove all docker containers

```bash
docker rm $(docker ps -aq)
```

### List all docker image ids
```bash
docker ps -aq
```

### Remove all docker images

```bash
docker rmi $(docker images -aq)
```

### Remove everything from docker

```bash
docker system prune
```
