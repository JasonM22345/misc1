import requests
import urllib.parse

# Proxmox credentials and URL
proxmox_ip = 'proxmox-host-ip'
username = 'root@pam'
password = 'your-password'
node = 'proxmox-node'
screenshot_path = screenshot_info['filename']

# Step 1: Authenticate and get ticket
auth_url = f'https://{proxmox_ip}:8006/api2/json/access/ticket'
auth_data = {
    'username': username,
    'password': password,
    'realm': 'pam'  # Adjust if needed
}

# Authenticate
auth_response = requests.post(auth_url, data=auth_data, verify=False)
auth_response.raise_for_status()  # Ensure we got a successful response

# Parse the authentication ticket and CSRF token
auth_info = auth_response.json()['data']
ticket = auth_info['ticket']
csrf_token = auth_info['CSRFPreventionToken']

# Step 2: Download the screenshot
# Create the download URL
encoded_path = urllib.parse.quote(screenshot_path)
download_url = f"https://{proxmox_ip}:8006{encoded_path}"

# Headers for authenticated requests
cookies = {'PVEAuthCookie': ticket}
headers = {'CSRFPreventionToken': csrf_token}

# Request the file from the Proxmox server
download_response = requests.get(download_url, headers=headers, cookies=cookies, verify=False)

# Save the screenshot to a local file
local_path = '/path/to/local/directory/screenshot.png'
with open(local_path, 'wb') as file:
    file.write(download_response.content)

print(f"Screenshot downloaded and saved to: {local_path}")