# Quick Start Guide - Drone Route Builder System

## Швидкий старт

### Крок 1: Встановлення залежностей

```bash
# Створіть віртуальне середовище (рекомендовано)
python -m venv venv

# Активуйте віртуальне середовище
# Windows:
venv\Scripts\activate
# Linux/Mac:
#source venv/bin/activate

# Встановіть всі залежності
pip install -r requirements.txt
```

### Крок 2: Запуск системи

#### Варіант A: Streamlit UI (рекомендовано для початку)

```bash
streamlit run app/streamlit_app.py
```

Система відкриється в браузері за адресою: **http://localhost:8501**

#### Варіант B: FastAPI Backend

```bash
# Варіант 1: Через uvicorn напряму
uvicorn app.main:app --reload

# Варіант 2: Через main.py
python main.py
```

API буде доступне за адресою: **http://localhost:8000**
API документація (Swagger): **http://localhost:8000/docs**

### Крок 3: Використання Streamlit UI

1. **Налаштуйте дрон:**
   - Введіть назву дрона
   - Встановіть параметри (швидкість, висота, батарея)

2. **Додайте цільові точки:**
   - Вручну: введіть координати в бічній панелі
   - Або імпортуйте з CSV/GeoJSON файлу

3. **Налаштуйте депо (стартову точку):**
   - Введіть координати депо

4. **Виберіть опції планування:**
   - Алгоритм: A*, Theta*, або D*
   - Grid Graph: ввімкніть для більш точного планування
   - VRP: для мультидронових місій
   - Genetic Algorithm: для оптимізації маршруту

5. **Опційно: Додайте погодні дані:**
   - Ввімкніть "Use Weather Data"
   - Натисніть "Fetch Weather Data"
   - Система отримає дані з Open Meteo API

6. **Плануйте маршрут:**
   - Натисніть "Plan Route"
   - Перегляньте результат на карті
   - Перевірте метрики та валідацію

7. **Експортуйте:**
   - Завантажте маршрут у форматі .plan або JSON

## Приклади використання

### Приклад 1: Простий маршрут (один дрон)

```python
from app.domain.mission import Mission
from app.domain.drone import Drone
from app.domain.waypoint import Waypoint
from app.orchestrator.mission_orchestrator import MissionOrchestrator

# Створіть дрон
drone = Drone(
    name="Drone 1",
    max_speed=15.0,
    max_altitude=120.0,
    min_altitude=10.0,
    battery_capacity=100.0,
    power_consumption=50.0
)

# Створіть місію
mission = Mission(
    name="Test Mission",
    drones=[drone],
    target_points=[
        Waypoint(50.0, 30.0, 50.0, "Target 1"),
        Waypoint(50.01, 30.01, 60.0, "Target 2"),
    ],
    depot=Waypoint(49.99, 29.99, 0.0, "Depot")
)

# Плануйте маршрут
orchestrator = MissionOrchestrator(mission)
routes = orchestrator.plan_mission(use_grid=True, algorithm="astar")

# Перегляньте результат
for drone_name, route in routes.items():
    print(f"Route for {drone_name}: {len(route.waypoints)} waypoints")
    if route.metrics:
        print(f"  Distance: {route.metrics.total_distance/1000:.2f} km")
        print(f"  Energy: {route.metrics.total_energy:.2f} Wh")
```

### Приклад 2: З погодними даними

```python
from app.weather.weather_provider import WeatherProvider
from datetime import datetime

# Отримайте погодні дані
weather_provider = WeatherProvider()
weather_data = {}

# Для кожного waypoint
for target in mission.target_points:
    weather = weather_provider.get_weather(
        target.latitude,
        target.longitude,
        target.altitude,
        datetime.now()
    )
    if weather:
        weather_data[(target.latitude, target.longitude)] = weather

# Плануйте з урахуванням погоди
orchestrator = MissionOrchestrator(mission, weather_data)
routes = orchestrator.plan_mission(use_weather=True)
```

### Приклад 3: Мультидрон з VRP

```python
# Створіть кілька дронів
drones = [
    Drone(name="Drone 1", max_speed=15.0, max_altitude=120.0, ...),
    Drone(name="Drone 2", max_speed=15.0, max_altitude=120.0, ...),
]

mission = Mission(
    name="Multi-Drone Mission",
    drones=drones,
    target_points=[...],  # Багато цільових точок
    depot=depot
)

# Використайте VRP для оптимального розподілу
orchestrator = MissionOrchestrator(mission)
routes = orchestrator.plan_mission(use_vrp=True)
```

## Опційно: Налаштування PostgreSQL/PostGIS

Якщо потрібно зберігати місії в базі даних:

### 1. Запуск через Docker (рекомендовано)

**Використовуйте docker-compose.yml:**

```bash
# Запустити базу даних
docker-compose up -d

# Перевірити статус
docker-compose ps
```

База даних `drone_routes` створюється автоматично з PostGIS розширенням.

**Альтернативно (без docker-compose):**
```bash
docker run --name drone-routes-db \
  -e POSTGRES_USER=postgres \
  -e POSTGRES_PASSWORD=postgres \
  -e POSTGRES_DB=drone_routes \
  -p 5432:5432 \
  -d postgis/postgis:15-3.3
```

### 2. Налаштуйте підключення

Встановіть змінну оточення:
```bash
# Windows (PowerShell)
$env:DATABASE_URL="postgresql://postgres:postgres@localhost:5432/drone_routes"

# Windows (CMD)
set DATABASE_URL=postgresql://postgres:postgres@localhost:5432/drone_routes

# Linux/Mac
export DATABASE_URL="postgresql://postgres:postgres@localhost:5432/drone_routes"
```

Або створіть файл `.env` в корені проєкту:
```
DATABASE_URL=postgresql://postgres:postgres@localhost:5432/drone_routes
```

### 3. Перевірка підключення

Запустіть Streamlit додаток - на головному екрані ви побачите статус підключення до БД.

Таблиці створюються автоматично при першому підключенні.

> 📖 **Детальна інструкція:** Дивіться [DOCKER_SETUP.md](DOCKER_SETUP.md) для повної документації з Docker

## Тестування системи

Запустіть тестовий скрипт:

```bash
python test_system.py
```

Це перевірить базову функціональність системи.

## Усунення проблем

### Помилка: ModuleNotFoundError

**Рішення:** Переконайтеся, що віртуальне середовище активоване і всі залежності встановлені:
```bash
pip install -r requirements.txt
```

### Помилка: Порт вже зайнятий

**Рішення:** Змініть порт:
```bash
# Streamlit
streamlit run app/streamlit_app.py --server.port 8502

# FastAPI
uvicorn app.main:app --port 8001
```

### Помилка підключення до Open Meteo API

**Рішення:** Перевірте інтернет-з'єднання. API працює без API ключа, але потребує інтернет.

### Помилка з OR-Tools

**Рішення:** OR-Tools може потребувати додаткових залежностей на деяких системах. Перевірте документацію OR-Tools для вашої ОС.

## Додаткові ресурси

- **API документація:** http://localhost:8000/docs (після запуску FastAPI)
- **Streamlit UI:** http://localhost:8501 (після запуску Streamlit)
- **Open Meteo API:** https://open-meteo.com/

## Підтримка

Для питань та проблем перевірте:
1. Логи в консолі
2. Валідацію маршрутів (може містити деталі помилок)
3. README.md для детальної документації

