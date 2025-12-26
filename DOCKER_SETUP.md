# Налаштування бази даних через Docker

## Швидкий старт

### 1. Запуск PostgreSQL з PostGIS

Використовуйте `docker-compose.yml` для зручного запуску:

```bash
# Запустити базу даних
docker-compose up -d

# Перевірити статус
docker-compose ps

# Переглянути логи
docker-compose logs -f postgres
```

### 2. Створення бази даних та розширення PostGIS

Після запуску контейнера, PostGIS розширення буде автоматично доступне. 
База даних `drone_routes` створюється автоматично.

Якщо потрібно створити базу вручну або перевірити:

```bash
# Підключитися до PostgreSQL
docker exec -it drone-routes-db psql -U postgres

# Або через docker-compose
docker-compose exec postgres psql -U postgres
```

У psql консолі:

```sql
-- Перевірити, чи існує база
\l

-- Підключитися до бази
\c drone_routes

-- Перевірити PostGIS розширення
SELECT PostGIS_version();

-- Якщо PostGIS не встановлено (малоймовірно), встановіть:
CREATE EXTENSION IF NOT EXISTS postgis;
```

### 3. Налаштування змінної оточення

**Рекомендовано: використовуйте `.env` файл**

Створіть файл `.env` в корені проєкту (або скопіюйте `.env.example`):

```bash
# Windows
copy .env.example .env

# Linux/Mac
cp .env.example .env
```

Файл `.env` вже містить правильні налаштування для Docker:
```
DATABASE_URL=postgresql://postgres:postgres@localhost:5432/drone_routes
```

Система автоматично завантажить змінні з `.env` файлу при запуску.

**Альтернативно: встановіть змінну оточення вручну**

**Windows (PowerShell):**
```powershell
$env:DATABASE_URL="postgresql://postgres:postgres@localhost:5432/drone_routes"
```

**Windows (CMD):**
```cmd
set DATABASE_URL=postgresql://postgres:postgres@localhost:5432/drone_routes
```

**Linux/Mac:**
```bash
export DATABASE_URL="postgresql://postgres:postgres@localhost:5432/drone_routes"
```

### 4. Перевірка підключення

Запустіть Streamlit додаток:

```bash
streamlit run app/streamlit_app.py
```

На головному екрані ви побачите:
- ✅ **Database connected** - якщо підключення успішне
- ⚠️ **Database not connected** - якщо є проблеми

## Керування контейнером

### Зупинити базу даних:
```bash
docker-compose stop
```

### Запустити знову:
```bash
docker-compose start
```

### Зупинити та видалити контейнер (зберігає дані):
```bash
docker-compose down
```

### Зупинити та видалити контейнер разом з даними:
```bash
docker-compose down -v
```

### Перезапустити:
```bash
docker-compose restart
```

## Альтернативний спосіб (без docker-compose)

Якщо не хочете використовувати docker-compose:

```bash
# Запустити контейнер
docker run --name drone-routes-db \
  -e POSTGRES_USER=postgres \
  -e POSTGRES_PASSWORD=postgres \
  -e POSTGRES_DB=drone_routes \
  -p 5432:5432 \
  -v postgres_data:/var/lib/postgresql/data \
  -d postgis/postgis:15-3.3

# Зупинити
docker stop drone-routes-db

# Запустити знову
docker start drone-routes-db

# Видалити
docker rm -f drone-routes-db
```

## Доступ до даних

Дані зберігаються в Docker volume `postgres_data`, тому вони зберігаються навіть після видалення контейнера.

Щоб видалити дані:
```bash
docker volume rm drone-route-builder-system_postgres_data
```

## Усунення проблем

### Помилка: порт 5432 вже зайнятий

Якщо у вас вже запущений PostgreSQL на порту 5432:

1. **Змінити порт в docker-compose.yml:**
```yaml
ports:
  - "5433:5432"  # Зовнішній порт 5433, внутрішній 5432
```

2. **Оновити DATABASE_URL:**
```
DATABASE_URL=postgresql://postgres:postgres@localhost:5433/drone_routes
```

### Помилка: контейнер не запускається

Перевірте логи:
```bash
docker-compose logs postgres
```

### Помилка підключення з додатку

1. Перевірте, чи контейнер запущений:
```bash
docker ps | grep drone-routes-db
```

2. Перевірте змінну оточення:
```bash
# Windows PowerShell
echo $env:DATABASE_URL

# Linux/Mac
echo $DATABASE_URL
```

3. Перевірте підключення вручну:
```bash
docker exec -it drone-routes-db psql -U postgres -d drone_routes -c "SELECT 1;"
```

## Структура бази даних

Після першого запуску додатку, таблиці створюються автоматично через SQLAlchemy:

- `missions` - місії
- `drones` - дрони
- `target_points` - цільові точки та депо
- `routes` - маршрути
- `route_waypoints` - waypoints маршрутів
- `constraints` - обмеження місій
- `no_fly_zones` - заборонені зони

