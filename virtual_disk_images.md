Here’s a Python script that implements the functionality you described. The script utilizes SSH to execute commands on a remote host and works with disk images for virtual USB drives. It also includes a simple command-line interface (CLI) using argparse.

import os
import subprocess
import paramiko
import argparse
from datetime import datetime


# Hardcoded username and password
USERNAME = "your_username"
PASSWORD = "your_password"

# Hardcoded directory to store disk images
DRIVE_DIRECTORY = "/path/to/drive/directory"

# Function to execute SSH command
def ssh_execute(host, command):
    try:
        # Initialize SSH client
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(host, username=USERNAME, password=PASSWORD)

        # Execute command
        stdin, stdout, stderr = ssh.exec_command(command)
        stdout_data = stdout.read().decode()
        stderr_data = stderr.read().decode()

        # Close the SSH client
        ssh.close()

        # Return the output and errors
        return stdout_data, stderr_data
    except Exception as e:
        print(f"Error connecting via SSH: {e}")
        return "", str(e)


# 1) Function to create a virtual USB drive
def create_virtual_usb_drive(host, disk_name, size_gb=1, fs="fat32"):
    # Ensure the disk directory exists
    ssh_execute(host, f"mkdir -p {DRIVE_DIRECTORY}")
    
    # Check if disk image already exists
    disk_path = os.path.join(DRIVE_DIRECTORY, disk_name)
    if os.path.exists(disk_path):
        print(f"Disk image {disk_name} already exists.")
        return

    # Create the disk image using dd
    dd_command = f"dd if=/dev/zero of={disk_path} bs=1M count={size_gb * 1024} status=progress"
    print(f"Creating disk image: {disk_name} ({size_gb} GB)...")
    stdout, stderr = ssh_execute(host, dd_command)
    if stderr:
        print(f"Error creating disk image: {stderr}")
        return

    # Create partition table for both Linux and Windows (MBR)
    partition_command = f"parted {disk_path} mklabel msdos"
    ssh_execute(host, partition_command)

    # Create a partition on the disk
    create_partition_command = f"parted {disk_path} mkpart primary fat32 0% 100%"
    ssh_execute(host, create_partition_command)

    # Format the disk as FAT32
    mkfs_command = f"mkfs.{fs} {disk_path}"
    stdout, stderr = ssh_execute(host, mkfs_command)
    if stderr:
        print(f"Error formatting the disk: {stderr}")
        return

    print(f"Disk image {disk_name} created successfully.")
    

# 2) Function to delete a virtual USB drive
def delete_virtual_usb_drive(host, disk_name):
    disk_path = os.path.join(DRIVE_DIRECTORY, disk_name)
    if not os.path.exists(disk_path):
        print(f"Disk image {disk_name} does not exist.")
        return

    # Delete the disk image
    delete_command = f"rm -f {disk_path}"
    stdout, stderr = ssh_execute(host, delete_command)
    if stderr:
        print(f"Error deleting disk image: {stderr}")
        return

    print(f"Disk image {disk_name} deleted successfully.")


# 3) Function to list virtual USB drives
def list_virtual_usb_drives(host):
    # List files in the disk directory
    list_command = f"ls -l {DRIVE_DIRECTORY}"
    stdout, stderr = ssh_execute(host, list_command)
    if stderr:
        print(f"Error listing disk images: {stderr}")
        return

    # Print the file details with creation/modification/access times
    print("Listing disk images:")
    print(stdout)


# Main function to parse command-line arguments
def main():
    parser = argparse.ArgumentParser(description="Proxmox USB Utility")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Subcommand for creating a virtual USB drive
    create_parser = subparsers.add_parser("create", help="Create a virtual USB drive")
    create_parser.add_argument("--host", required=True, help="Remote host")
    create_parser.add_argument("--disk_name", required=True, help="Name of the disk image")
    create_parser.add_argument("--size", type=int, default=1, help="Size of the disk image in GB (default: 1 GB)")
    create_parser.add_argument("--fs", choices=["fat32", "ntfs", "ext4"], default="fat32", help="File system (default: fat32)")

    # Subcommand for deleting a virtual USB drive
    delete_parser = subparsers.add_parser("delete", help="Delete a virtual USB drive")
    delete_parser.add_argument("--host", required=True, help="Remote host")
    delete_parser.add_argument("--disk_name", required=True, help="Name of the disk image")

    # Subcommand for listing virtual USB drives
    list_parser = subparsers.add_parser("list", help="List virtual USB drives")
    list_parser.add_argument("--host", required=True, help="Remote host")

    args = parser.parse_args()

    if args.command == "create":
        create_virtual_usb_drive(args.host, args.disk_name, args.size, args.fs)
    elif args.command == "delete":
        delete_virtual_usb_drive(args.host, args.disk_name)
    elif args.command == "list":
        list_virtual_usb_drives(args.host)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

How it works:

	1.	Create Virtual USB Drive (create):
	•	It first checks if the directory exists and creates it if not.
	•	It checks if a disk with the same name exists.
	•	Uses dd to create a virtual disk, parted to create a partition table, and mkfs to format it (defaulting to FAT32).
	•	The disk is not mounted automatically.
	2.	Delete Virtual USB Drive (delete):
	•	Checks if the disk exists and then deletes it.
	3.	List Virtual USB Drives (list):
	•	Lists all files in the specified directory with details (modified, accessed, created).

Command-line Usage:

# Create a virtual USB drive
proxmox_usb_utils create --host <host_ip> --disk_name <disk_name> --size 2 --fs fat32

# Delete a virtual USB drive
proxmox_usb_utils delete --host <host_ip> --disk_name <disk_name>

# List virtual USB drives
proxmox_usb_utils list --host <host_ip>

Requirements:

	•	Python 3
	•	paramiko library for SSH communication (pip install paramiko)

Notes:

	•	The script does not mount the created disk image.
	•	The user must configure the correct path to the disk directory (DRIVE_DIRECTORY).
	•	Make sure SSH is accessible from the script to the target host.

This solution offers a flexible and easy way to manage virtual USB drives remotely via SSH.
