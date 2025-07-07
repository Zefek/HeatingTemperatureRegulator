import openai
import os
import requests

# Nastavení OpenAI
openai.api_key = os.getenv("OPENAI_API_KEY")

# Načti diff PR ze souboru nebo API
with open("diff.patch", "r", encoding="utf-8") as f:
    pr_diff = f.read()

# Vytvoř požadavek na GPT-4
response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "Jsi zkušený reviewer kódu. Zhodnoť změny v PR."},
        {"role": "user", "content": f"Zde je diff pull requestu:\n\n{pr_diff}"}
    ],
    temperature=0.3
)

review = response["choices"][0]["message"]["content"]

# Odeslání komentáře zpět do GitHub PR
headers = {
    "Authorization": f"Bearer {os.getenv('GITHUB_TOKEN')}",
    "Accept": "application/vnd.github.v3+json"
}
repo = os.getenv("GITHUB_REPOSITORY")
pr_number = os.getenv("GITHUB_REF").split("/")[-1]

response = requests.post(
    f"https://api.github.com/repos/{repo}/issues/{pr_number}/comments",
    headers=headers,
    json={"body": review}
)

print("Komentář přidán:", response.status_code)
