#!/bin/bash
# Setup systemd service for Jetson hardware bridge
# Auto-starts on boot with Restart=always policy

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="rescue-robot-bridge"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
PYTHON_EXEC=$(which python3)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up systemd service for Rescue Robot Hardware Bridge...${NC}"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}Please run as root (use sudo)${NC}"
    exit 1
fi

# Check if Python3 is available
if [ -z "$PYTHON_EXEC" ]; then
    echo -e "${RED}Error: python3 not found${NC}"
    exit 1
fi

# Get the actual user who ran sudo (for log file permissions)
ACTUAL_USER="${SUDO_USER:-$USER}"

# Create systemd service file
echo -e "${YELLOW}Creating systemd service file...${NC}"
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Rescue Robot Hardware Bridge (ZMQ + ROS2)
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=${ACTUAL_USER}
WorkingDirectory=${PROJECT_ROOT}
Environment="PATH=${PATH}"
Environment="PYTHONPATH=${PROJECT_ROOT}"
ExecStart=${PYTHON_EXEC} ${PROJECT_ROOT}/jetson/hardware_bridge.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}

# Resource limits (optional)
LimitNOFILE=65536

# Security hardening (optional)
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
EOF

echo -e "${GREEN}Service file created: ${SERVICE_FILE}${NC}"

# Reload systemd daemon
echo -e "${YELLOW}Reloading systemd daemon...${NC}"
systemctl daemon-reload

# Enable service to start on boot
echo -e "${YELLOW}Enabling service to start on boot...${NC}"
systemctl enable "${SERVICE_NAME}.service"

# Check if user wants to start service now
read -p "Start the service now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Starting service...${NC}"
    systemctl start "${SERVICE_NAME}.service"
    echo -e "${GREEN}Service started!${NC}"
    echo -e "${YELLOW}View logs with: journalctl -u ${SERVICE_NAME} -f${NC}"
else
    echo -e "${GREEN}Service installed but not started. Start manually with:${NC}"
    echo -e "  ${YELLOW}sudo systemctl start ${SERVICE_NAME}${NC}"
fi

echo -e "\n${GREEN}Setup complete!${NC}"
echo -e "\nUseful commands:"
echo -e "  Start service:   ${YELLOW}sudo systemctl start ${SERVICE_NAME}${NC}"
echo -e "  Stop service:    ${YELLOW}sudo systemctl stop ${SERVICE_NAME}${NC}"
echo -e "  Restart service: ${YELLOW}sudo systemctl restart ${SERVICE_NAME}${NC}"
echo -e "  View logs:       ${YELLOW}journalctl -u ${SERVICE_NAME} -f${NC}"
echo -e "  Disable service: ${YELLOW}sudo systemctl disable ${SERVICE_NAME}${NC}"