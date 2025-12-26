@echo off
REM Batch script для запуску Streamlit на Windows

REM Перейдіть до папки проєкту
cd /d "%~dp0"

REM Активуйте віртуальне середовище
call venv\Scripts\activate.bat

REM Запустіть Streamlit
streamlit run app/streamlit_app.py

pause

