import openai
import os
import requests

# Nastavení OpenAI
openai.api_key = os.getenv(OPENAI_API_KEY)

# Získání dat o PR z GitHubu (pomocí GitHub Actions se předá kontext, nebo použij GraphQL REST API)
pr_diff = open(diff.patch).read()  # nebo načíst pomocí API

# Dotaz na GPT
response = openai.ChatCompletion.create(
    model=gpt-4,
    messages=[
        {role system, content Jsi zkušený reviewer kódu. Zhodnoť změny v PR.},
        {role user, content fZde je diff pull requestunn{pr_diff}}
    ],
    temperature=0.3
)

review = response[choices][0][message][content]

# Odeslání komentáře zpět do GitHub PR
headers = {
    Authorization fBearer {os.getenv('GITHUB_TOKEN')},
    Accept applicationvnd.github.v3+json
}
repo = os.getenv(GITHUB_REPOSITORY)
pr_number = os.getenv(GITHUB_REF).split()[-1]

requests.post(
    fhttpsapi.github.comrepos{repo}issues{pr_number}comments,
    headers=headers,
    json={body review}
)