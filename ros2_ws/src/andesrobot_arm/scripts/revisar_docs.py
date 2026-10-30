#!/usr/bin/env python3
"""Revisa la documentación del repo (enlaces, figuras, fórmulas, conteo de pruebas).

  python3 ros2_ws/src/andesrobot_arm/scripts/revisar_docs.py [raíz del repo]

Qué revisa y por qué: andesrobot_arm/revisar_docs.py. Sale con código 1 si encuentra errores
(así lo usa el CI de GitHub Actions, .github/workflows/ci.yml).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from andesrobot_arm.revisar_docs import revisar  # noqa: E402


def main():
    raiz = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
    errores = revisar(raiz)
    for e in errores:
        print('ERROR', e)
    print(f'{len(errores)} error(es) en la documentación de {raiz}')
    sys.exit(1 if errores else 0)


if __name__ == '__main__':
    main()
