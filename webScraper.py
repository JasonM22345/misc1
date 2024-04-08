import requests
from bs4 import BeautifulSoup

# Requesting target website
dunkin = requests.get("https://www.dunkindonuts.com/en/locations?location=20005")

# Parsing website HTML
soup = BeautifulSoup(dunkin.text, "html.parser")

# Open a file in write mode
with open("dunkin_page_content.txt", "w", encoding="utf-8") as file:
    # Write the entire page content to the file
    file.write(soup.prettify())

