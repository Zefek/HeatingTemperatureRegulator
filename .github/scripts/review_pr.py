import os
import requests
from openai import OpenAI

# Inicializace klienta
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Načti diff změny v pull requestu
with open("diff.patch", "r", encoding="utf-8") as f:
    pr_diff = f.read()

# Vytvoř požadavek na GPT-4
response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "Jsi zkušený reviewer. Zhodnoť změny v PR."},
        {"role": "user", "content": f"Zde jsou změny v kódu:\n\n{pr_diff}"}
    ],
    temperature=0.3
)

# Získání odpovědi
review = response.choices[0].message.content

# Připrav hlavičky a data pro komentář na GitHub
headers = {
    "Authorization": f"Bearer {os.getenv('GITHUB_TOKEN')}",
    "Accept": "application/vnd.github.v3+json"
}

repo = os.getenv("GITHUB_REPOSITORY")
pr_number = os.getenv("GITHUB_REF").split("/")[2]

# Odeslání komentáře do PR
comment_response = requests.post(
    f"https://api.github.com/repos/{repo}/issues/{pr_number}/comments",
    headers=headers,
    json={"body": review}
)

print("Komentář přidán:", comment_response.status_code)
