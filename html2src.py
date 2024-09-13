
import re


with open('index.html', encoding='utf-8') as fp:
    lastrow = None
    for line in fp:
        if line.strip().startswith('//'):
            continue
        if lastrow is None:
            html = '  const char *html = "'
        else:
            cur = lastrow
            cur = cur.replace('\n', '\\\\n\\\\\n')
            html = html + cur
        lastrow = line.replace('"', "'")
cur = lastrow.replace('\n', '";')
html = html + cur

with open('src/main.cpp', encoding='utf-8') as fp:
    src = fp.read()

html = re.sub(r' *const char \*html = "[^"]*";', html, src, flags=re.DOTALL)
with open("src/main.cpp", 'w', encoding='utf-8') as fp:
    fp.write(html)
