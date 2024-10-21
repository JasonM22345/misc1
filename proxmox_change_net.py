from proxmoxer import ProxmoxAPI

# Function to update the network configuration of the VM
def update_vm_network(vm_obj, new_bridge):
    # Connect to the Proxmox API
    proxmox = ProxmoxAPI('proxmox.example.com', user='root@pam', password='your_password', verify_ssl=False)
    
    # Extract VM details
    node = vm_obj['node']
    vm_id = vm_obj['vmid']
    
    # Get the current VM configuration
    vm_config = proxmox.nodes(node).qemu(vm_id).config.get()
    
    # Check if net0 exists in the configuration
    if 'net0' in vm_config:
        # Get the current net0 configuration
        current_net_config = vm_config['net0']
        
        # Find and replace the existing bridge=... part with the new bridge
        new_net_config = ''
        for setting in current_net_config.split(','):
            if setting.startswith('bridge='):
                new_net_config += f"bridge={new_bridge},"
            else:
                new_net_config += setting + ','
        
        # Remove trailing comma
        new_net_config = new_net_config.rstrip(',')

        # Apply the updated network configuration for net0
        proxmox.nodes(node).qemu(vm_id).config.put(net0=new_net_config)

        print(f"VM {vm_id} on node {node} updated to use bridge {new_bridge}.")
    else:
        print(f"Error: net0 configuration not found for VM {vm_id}.")

# Example usage
vm_obj = {'node': 'pve', 'vmid': 100}  # Example VM object
new_bridge = 'vmbr1'  # New bridge to assign

update_vm_network(vm_obj, new_bridge)