from proxmoxer import ProxmoxAPI

def is_vm_on_zfs(proxmox, node, vmid):
    """
    Check if at least one disk on the given VM is using ZFS storage.
    
    :param proxmox: ProxmoxAPI instance
    :param node: Proxmox node name (e.g., 'pve')
    :param vmid: VM ID
    :return: True if at least one disk is on ZFS, otherwise False
    """
    # Get VM configuration
    vm_config = proxmox.nodes(node).qemu(vmid).config.get()

    # Disk prefixes to check (scsi, sata, ide, virtio)
    disk_prefixes = ['scsi', 'sata', 'ide', 'virtio']

    for prefix in disk_prefixes:
        for i in range(10):  # Checking up to 10 disks per type (adjust if needed)
            disk_key = f"{prefix}{i}"
            if disk_key in vm_config:
                disk_info = vm_config[disk_key]
                storage_name = disk_info.split(":")[0]  # Extract storage name

                # Get storage configuration
                storage_config = proxmox.nodes(node).storage(storage_name).get()

                # Check if storage type is ZFS
                if storage_config.get('type') == 'zfspool':
                    return True

    return False

# Example Usage
proxmox = ProxmoxAPI("proxmox_host", user="root@pam", password="your_password", verify_ssl=False)
node_name = "pve"  # Replace with your node name
vm_id = 100  # Replace with your VM ID

if is_vm_on_zfs(proxmox, node_name, vm_id):
    print("At least one disk on the VM is on ZFS.")
else:
    print("No disks on the VM are using ZFS.")
