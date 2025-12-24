"""Main entry point for the Drone Route Builder System.

Run FastAPI server:
    uvicorn app.main:app --reload

Run Streamlit UI:
    streamlit run app/streamlit_app.py
"""
if __name__ == '__main__':
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
