import os
import urllib.request
from PIL import Image

def download_and_convert_to_png(source_url, destination_path):
    try:
        # Step 1: Download the file using urllib
        file_name = os.path.basename(source_url)
        downloaded_file = os.path.join(destination_path, file_name)

        print(f"Downloading file from {source_url}...")
        urllib.request.urlretrieve(source_url, downloaded_file)
        print(f"File downloaded: {downloaded_file}")

        # Step 2: Open the downloaded file with PIL
        print(f"Converting {downloaded_file} to PNG...")
        with Image.open(downloaded_file) as img:
            # Step 3: Define the output PNG file path
            png_file = os.path.join(destination_path, os.path.splitext(file_name)[0] + '.png')

            # Step 4: Convert and save the file as PNG
            img.save(png_file, 'PNG')
            print(f"File successfully converted and saved as {png_file}")

        # Clean up by removing the original downloaded file if needed
        os.remove(downloaded_file)
        print(f"Cleaned up the original file: {downloaded_file}")

    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
# download_and_convert_to_png('https://example.com/image.jpg', '/path/to/destination')