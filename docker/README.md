Docker Deployment

Files with `host` on the end are ment to be run from the host.
Files with `cont` on the end are ment to be run on the container.

The `create_host.sh` will create a container sharing port `8888` for Jupyter notebooks, the script will also enable user interfaces and mount the container's `/home/ros` on the folder `~/ros_docker`. So run:

On host:
```bash
mkdir ~/ros_docker
bash create_host.sh
```