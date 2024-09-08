container_id=$(docker ps -q | head -n 1)

if [ -n "$container_id" ]; then
    echo "Container ID: $container_id"
    docker exec -it $container_id /bin/bash
else
    echo "No running containers found."
fi
