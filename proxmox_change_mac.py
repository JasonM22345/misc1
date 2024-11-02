import re
from proxmoxer import ProxmoxAPI

# Function to update the MAC addresses of the VM's network interfaces
def update_vm_mac_addresses(vm_obj, new_macs):
    # Connect to the Proxmox API
    proxmox = ProxmoxAPI('proxmox.example.com', user='root@pam', password='your_password', verify_ssl=False)
    
    # Extract VM details
    node = vm_obj['node']
    vm_id = vm_obj['vmid']
    
    # Get the current VM configuration
    vm_config = proxmox.nodes(node).qemu(vm_id).config.get()
    
    # Find all network interfaces in the VM configuration
    net_interfaces = [key for key in vm_config if key.startswith('net')]
    
    # Verify the number of MAC addresses matches the number of network interfaces
    if len(new_macs) != len(net_interfaces):
        raise ValueError("The number of MAC addresses provided (%d) does not match the number of network interfaces (%d)." % (len(new_macs), len(net_interfaces)))
    
    # Loop through each network interface and update the MAC address
    for i, net_interface in enumerate(net_interfaces):
        current_net_config = vm_config[net_interface]
        
        # Use regex to find the model and replace the MAC address part only
        # Match format: model=XX:XX:XX:XX:XX:XX
        new_net_config = re.sub(r'(^[^=]+)=([0-9a-fA-F:]{17})', r'\1=%s' % new_macs[i], current_net_config)
        
        # Apply the updated network configuration
        proxmox.nodes(node).qemu(vm_id).config.put(**{net_interface: new_net_config})
        
        print("Updated %s of VM %d on node %s with MAC address %s." % (net_interface, vm_id, node, new_macs[i]))

# Example usage
vm_obj = {'node': 'pve', 'vmid': 100}  # Example VM object
new_macs = ['02:00:00:00:00:01', '02:00:00:00:00:02']  # New MAC addresses for each network interface
update_vm_mac_addresses(vm_obj, new_macs)