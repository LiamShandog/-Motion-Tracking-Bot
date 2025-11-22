#!/bin/bash
# Docker build and deployment script for motion_tracking_bot on Raspberry Pi 5

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Motion Tracking Bot - Docker Setup${NC}"
echo "======================================"

# Parse arguments
COMMAND=${1:-help}
TARGET_HOST=${2:-pi@192.168.1.100}

case $COMMAND in
  build)
    echo -e "${GREEN}Building Docker image for ARM64 (Raspberry Pi 5)...${NC}"
    docker buildx build --platform linux/arm64 -t motion_tracking_bot:pi5 -f Dockerfile .
    echo -e "${GREEN}Build complete!${NC}"
    ;;

  build-local)
    echo -e "${GREEN}Building Docker image locally (native architecture)...${NC}"
    docker build -t motion_tracking_bot:pi5 -f Dockerfile .
    echo -e "${GREEN}Build complete!${NC}"
    ;;

  run)
    echo -e "${GREEN}Running container locally...${NC}"
    docker-compose up --build
    ;;

  run-detached)
    echo -e "${GREEN}Running container in detached mode...${NC}"
    docker-compose up -d
    echo -e "${GREEN}Container started. Use 'docker-compose logs -f' to view logs.${NC}"
    ;;

  stop)
    echo -e "${YELLOW}Stopping containers...${NC}"
    docker-compose down
    echo -e "${GREEN}Containers stopped.${NC}"
    ;;

  logs)
    echo -e "${YELLOW}Showing container logs...${NC}"
    docker-compose logs -f
    ;;

  shell)
    echo -e "${GREEN}Opening shell in running container...${NC}"
    docker-compose exec motion_tracking_bot bash
    ;;

  save)
    echo -e "${GREEN}Saving image to tar file...${NC}"
    docker save motion_tracking_bot:pi5 | gzip > motion_tracking_bot_pi5.tar.gz
    echo -e "${GREEN}Image saved to motion_tracking_bot_pi5.tar.gz${NC}"
    ;;

  deploy)
    echo -e "${GREEN}Deploying to Raspberry Pi at $TARGET_HOST...${NC}"
    echo "1. Saving image..."
    docker save motion_tracking_bot:pi5 | gzip > motion_tracking_bot_pi5.tar.gz
    
    echo "2. Copying to Pi..."
    scp motion_tracking_bot_pi5.tar.gz $TARGET_HOST:/tmp/
    
    echo "3. Loading on Pi..."
    ssh $TARGET_HOST << 'EOF'
      docker load < /tmp/motion_tracking_bot_pi5.tar.gz
      docker-compose up -d
      echo "Deployment complete!"
EOF
    echo -e "${GREEN}Deployment complete!${NC}"
    ;;

  clean)
    echo -e "${YELLOW}Cleaning up containers and images...${NC}"
    docker-compose down -v
    docker rmi motion_tracking_bot:pi5 || true
    rm -f motion_tracking_bot_pi5.tar.gz
    echo -e "${GREEN}Cleanup complete!${NC}"
    ;;

  *)
    echo "Usage: $0 <command> [target_host]"
    echo ""
    echo "Commands:"
    echo "  build              Build Docker image for ARM64 (cross-compile)"
    echo "  build-local        Build Docker image locally (native arch)"
    echo "  run                Run container in foreground"
    echo "  run-detached       Run container in background"
    echo "  stop               Stop running container"
    echo "  logs               View container logs"
    echo "  shell              Open shell in container"
    echo "  save               Save image to tar.gz file"
    echo "  deploy [host]      Deploy to Raspberry Pi (default: pi@192.168.1.100)"
    echo "  clean              Remove containers and images"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 run"
    echo "  $0 deploy pi@192.168.1.50"
    ;;
esac
