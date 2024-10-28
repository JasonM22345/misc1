import proxmoxer

def create_clone(proxmox_vm, clone_name, snapshot_name=None, linked=False):
    """
    Clones a Proxmox VM, optionally from a snapshot and as a linked clone.

    Parameters:
        proxmox_vm: The Proxmox VM object.
        clone_name: The name of the clone to be created.
        snapshot_name: The name of the snapshot to base the clone on (optional).
        linked: Boolean indicating whether to create a linked clone.

    Returns:
        bool: True if the clone was created successfully.
    """
    # Get the VM ID and node from the VM object
    vm_id = proxmox_vm['vmid']
    node = proxmox_vm['node']

    # Set up the clone configuration
    clone_config = {
        "newid": proxmox_vm["vmid"] + 1,  # or specify another unique ID
        "name": clone_name,
        "target": node,
    }

    # Add optional parameters for snapshot and linked clone
    if snapshot_name:
        clone_config["snapname"] = snapshot_name
    if linked:
        clone_config["full"] = 0  # 'full' set to 0 creates a linked clone
    else:
        clone_config["full"] = 1  # 'full' set to 1 creates a full clone

    # Execute the clone command
    proxmox = proxmoxer.ProxmoxAPI("proxmox_host", user="your_username", password="your_password", verify_ssl=False)
    proxmox.nodes(node).qemu(vm_id).clone.create(**clone_config)
    
    return True