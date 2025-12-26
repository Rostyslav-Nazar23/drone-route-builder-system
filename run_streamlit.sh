#!/bin/bash
# Shell script для запуску Streamlit на Linux/Mac

# Перейдіть до папки проєкту
cd "$(dirname "$0")"

# Активуйте віртуальне середовище
source venv/bin/activate

# Запустіть Streamlit
streamlit run app/streamlit_app.py

