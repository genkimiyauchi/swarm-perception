import os
import re

# Path to the template file (trial1.html)
template_file = 'templates/trial1.html'

# Read the template content
with open(template_file, 'r') as f:
    template = f.read()

for i in range(2, 21):
    new_filename = f'templates/trial{i}.html'
    content = template

    # 1. Replace <h1>Trial 1</h1> with <h1>Trial i</h1>
    content = re.sub(r'<h1>Trial \d+</h1>', f'<h1>Trial {i}</h1>', content)

    # 2. Replace <form action="/startpage" method="get"> with previous trial (i-1)
    content = re.sub(r'<form action="/startpage" method="get">', f'<form action="/trial{i-1}" method="get">', content)

    # 2. Replace <form action="/endpage" method="get"> with next trial (i+1)
    content = re.sub(r'<form action="/endpage" method="get">', f'<form action="/trial{i+1}" method="get">', content, count=1)

    # 4. Replace <p">2 of 22</p> with <p">{i+1} of 22</p>
    content = re.sub(r'<p">\d+ of 22</p>', f'<p">{i+1} of 22</p>', content)

    with open(new_filename, 'w') as out:
        out.write(content)
    print(f'Created {new_filename}')
