from proxmoxer import ProxmoxAPI

# Proxmox server connection details
PROXMOX_HOST = 'your_proxmox_host'
USERNAME = 'root@pam'
PASSWORD = 'your_password'
VERIFY_SSL = False  # Set to True if you have valid SSL certificates

# VM IDs
SOURCE_VM_ID = 100
CLONE_VM_ID = 101

# Connect to Proxmox API
proxmox = ProxmoxAPI(PROXMOX_HOST, user=USERNAME, password=PASSWORD, verify_ssl=VERIFY_SSL)

# Step 1: Clone VM 100 to VM 101
def clone_vm(source_vm, clone_vm):
    node = proxmox.nodes.get()[0]['node']  # Assuming a single node setup
    print(f"Cloning VM {source_vm} to VM {clone_vm}...")
    
    proxmox.nodes(node).qemu(source_vm).clone.create(
        newid=clone_vm,
        name=f"vm-{clone_vm}",
        full=1  # Full clone to ensure independent snapshots
    )
    print(f"VM {source_vm} successfully cloned to VM {clone_vm}.")

# Step 2: Retrieve snapshots from VM 100
def get_snapshots(vm_id):
    node = proxmox.nodes.get()[0]['node']
    snapshots = proxmox.nodes(node).qemu(vm_id).snapshot.get()
    snapshot_dict = {snap['name']: snap for snap in snapshots}
    return snapshot_dict

# Step 3: Create snapshots recursively on VM 101
def create_snapshots_on_clone(source_snapshots, clone_vm):
    node = proxmox.nodes.get()[0]['node']

    # Helper function to recursively create snapshots
    def create_snapshot_recursive(snapshot_name):
        snapshot = source_snapshots[snapshot_name]
        parent = snapshot.get('parent')

        # Create parent snapshot first if it exists
        if parent:
            if parent not in created_snapshots:
                create_snapshot_recursive(parent)
        
        # Create the current snapshot
        print(f"Creating snapshot '{snapshot_name}' on VM {clone_vm}...")
        proxmox.nodes(node).qemu(clone_vm).snapshot.create(
            snapname=snapshot_name,
            description=snapshot.get('description', ''),
            vmstate=snapshot.get('vmstate', False)
        )
        created_snapshots.add(snapshot_name)
        print(f"Snapshot '{snapshot_name}' created successfully.")

    created_snapshots = set()

    # Start from the 'current' pseudo state
    if 'current' in source_snapshots:
        create_snapshot_recursive('current')
    else:
        for snapshot_name in source_snapshots:
            if snapshot_name not in created_snapshots:
                create_snapshot_recursive(snapshot_name)

# Main execution flow
if __name__ == "__main__":
    clone_vm(SOURCE_VM_ID, CLONE_VM_ID)
    
    source_snapshots = get_snapshots(SOURCE_VM_ID)
    if not source_snapshots:
        print(f"No snapshots found on VM {SOURCE_VM_ID}.")
    else:
        create_snapshots_on_clone(source_snapshots, CLONE_VM_ID)
        print(f"All snapshots copied from VM {SOURCE_VM_ID} to VM {CLONE_VM_ID}.")