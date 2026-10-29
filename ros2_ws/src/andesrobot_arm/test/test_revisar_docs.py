# Pruebas de revisar_docs.py: cada error que detecta, con documentos de juguete en tmp_path.
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_revisar_docs.py
import os

from andesrobot_arm.revisar_docs import revisar, slug


def escribir(raiz, ruta, texto):
    destino = os.path.join(raiz, ruta)
    os.makedirs(os.path.dirname(destino), exist_ok=True)
    with open(destino, 'w', encoding='utf-8') as f:
        f.write(texto)


def pruebas(raiz, paquete, n):
    escribir(raiz, f'ros2_ws/src/{paquete}/test/test_algo.py',
             ''.join(f'def test_{i}():\n    pass\n' for i in range(n)))


def test_slug_de_github():
    assert slug('## 5. Cadena cinemática'.lstrip('# ')) == '5-cadena-cinemática'
    assert slug('`pose` y «frame»') == 'pose-y-frame'


def test_documentos_sanos_no_dan_errores(tmp_path):
    raiz = str(tmp_path)
    escribir(raiz, 'a.md',
             '# Uno\n\n## Sección dos\n\nVer [dos](#sección-dos) y [b](docs/b.md#x).\n')
    escribir(raiz, 'docs/b.md', '<a id="x"></a>\n# B\n')
    assert revisar(raiz) == []


def test_enlace_y_ancla_rotos(tmp_path):
    raiz = str(tmp_path)
    escribir(raiz, 'a.md', 'Ver [nada](no_existe.md) y [b](docs/b.md#falta) y [yo](#tampoco).\n')
    escribir(raiz, 'docs/b.md', '# B\n')
    errores = revisar(raiz)
    assert len(errores) == 3
    assert any('no_existe.md' in e for e in errores)


def test_operatorname_se_rechaza(tmp_path):
    escribir(str(tmp_path), 'a.md', 'La traza: $`\\operatorname{tr} R`$\n')
    assert any('operatorname' in e for e in revisar(str(tmp_path)))


def test_figuras_y_secciones_del_informe_en_orden(tmp_path):
    raiz = str(tmp_path)
    escribir(raiz, 'docs/informe/README.md',
             '## 1. Uno\n\n**Figura 1.** a\n\n## 3. Tres\n\n**Figura 3.** b\n\n**Figura 2.** c\n')
    errores = revisar(raiz)
    assert any('figuras' in e for e in errores) and any('secciones' in e for e in errores)


def test_conteo_de_pruebas_contra_el_codigo(tmp_path):
    raiz = str(tmp_path)
    pruebas(raiz, 'andesrobot_safety', 3)
    escribir(raiz, 'a.md', 'Resultado: **3 de 3** en `andesrobot_safety`.\n')
    assert revisar(raiz) == []
    escribir(raiz, 'a.md', 'Resultado: **9 de 9** en `andesrobot_safety`.\n')
    assert any('el código tiene 3' in e for e in revisar(raiz))
    escribir(raiz, 'a.md', 'Resultado: **2 de 3** en `andesrobot_safety`.\n')
    assert any('no pasan todas' in e for e in revisar(raiz))
