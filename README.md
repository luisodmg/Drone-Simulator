# Simulacion y Diagrama de Cuadrirotor

Este proyecto contiene:

- `quadrotorSim.py`: simulacion dinamica de un cuadrirotor con control PD en altura.
- `diagram.py`: generacion del diagrama de bloques del sistema de control.

## Requisitos

- Python 3.10 o superior
- Dependencias en `requirements.txt`

## Instalacion

```bash
pip install -r requirements.txt
```

## Ejecucion

1. Ejecutar la simulacion:

```bash
python quadrotorSim.py
```

2. Generar el diagrama:

```bash
python diagram.py
```

Al ejecutar `diagram.py`, se guarda automaticamente la imagen:

- `diagrama_cuadrirotor.png`

## Dependencias

- numpy
- scipy
- matplotlib
