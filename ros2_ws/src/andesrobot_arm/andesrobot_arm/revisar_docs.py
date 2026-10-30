"""Revisión automática de la documentación (sin ROS ni red: solo biblioteca estándar).

Busca los errores que una persona no ve a simple vista y que ya se colaron una vez:
  1. Enlaces entre archivos .md rotos, y anclas (#sección) que no existen.
  2. Figuras del informe fuera de orden o repetidas («**Figura 8.**» antes de la 7).
  3. Secciones numeradas del informe (## 1., ## 2., …) que se saltan un número.
  4. Fórmulas con `\\operatorname`, que el renderizador de GitHub rechaza («The following macros
     are not allowed: operatorname»): hay que escribir \\mathrm{…}.
  5. Un «**N de N** en `X`» (pruebas que pasan) que no coincide con las pruebas que tiene el
     código: X es un paquete (cuenta las `def test_` de su carpeta test/) o un archivo test_*.py.

No revisa enlaces a páginas web (no usa red) ni que las pruebas realmente pasen: eso lo hace
pytest.
Uso: python3 scripts/revisar_docs.py [raíz del repo]   (sale con error si encuentra algo)
"""
import os
import re

OMITIR = {'.git', 'node_modules', 'build', 'install', 'log', 'pdf', 'capturas', 'datasets'}


def archivos_md(raiz):
    for carpeta, dirs, archivos in os.walk(raiz):
        dirs[:] = [d for d in dirs if d not in OMITIR]
        for nombre in archivos:
            if nombre.endswith('.md'):
                yield os.path.join(carpeta, nombre)


def sin_codigo(texto):
    """El texto sin bloques ``` (ahí hay ejemplos que no son enlaces ni fórmulas reales)."""
    return re.sub(r'```.*?```', '', texto, flags=re.S)


def formulas(texto):
    """Solo las fórmulas: $`…`$, $$…$$ y bloques ```math (no las menciones en el texto)."""
    en_linea = re.findall(r'\$`(.*?)`\$', texto, flags=re.S)
    dobles = re.findall(r'\$\$(.*?)\$\$', texto, flags=re.S)
    bloques = re.findall(r'```math\n(.*?)```', texto, flags=re.S)
    return '\n'.join(en_linea + dobles + bloques)


def slug(titulo):
    """El ancla que GitHub le pone a un título: minúsculas, sin signos, espacios → guiones."""
    titulo = re.sub(r'`|\*|\[|\]|<[^>]+>', '', titulo).strip().lower()
    return re.sub(r'\s', '-', re.sub(r'[^\w\s-]', '', titulo))


def anclas(texto):
    """Todas las anclas válidas de un .md: ids explícitos y títulos (con -1, -2 si se repiten)."""
    texto = sin_codigo(texto)
    out = set(re.findall(r'<a id="([^"]+)"', texto))
    vistos = {}
    for titulo in re.findall(r'^#{1,6} +(.*)$', texto, flags=re.M):
        base = slug(titulo)
        n = vistos.get(base, 0)
        vistos[base] = n + 1
        out.add(base if n == 0 else f'{base}-{n}')
    return out


def revisar_enlaces(ruta, texto, cache):
    errores = []
    for destino in re.findall(r'\]\(([^)\s]+)\)', sin_codigo(texto)):
        if destino.startswith(('http://', 'https://', 'mailto:')):
            continue
        archivo, _, ancla = destino.partition('#')
        objetivo = os.path.normpath(os.path.join(os.path.dirname(ruta), archivo)) \
            if archivo else ruta
        if not os.path.exists(objetivo):
            errores.append(f'{ruta}: enlace roto a «{archivo}»')
            continue
        if ancla and objetivo.endswith('.md'):
            if objetivo not in cache:
                cache[objetivo] = anclas(open(objetivo, encoding='utf-8').read())
            if ancla not in cache[objetivo]:
                errores.append(f'{ruta}: el ancla «#{ancla}» no existe en {objetivo}')
    return errores


def revisar_informe(ruta, texto):
    """Orden de las figuras numeradas y de las secciones numeradas."""
    errores = []
    figuras = [int(n) for n in re.findall(r'^\*\*Figura (\d+)\.\*\*', texto, flags=re.M)]
    if figuras != list(range(1, len(figuras) + 1)):
        errores.append(f'{ruta}: las figuras numeradas deben ir 1, 2, 3… en orden; hay {figuras}')
    secciones = [int(n) for n in re.findall(r'^## (\d+)\. ', texto, flags=re.M)]
    if secciones != list(range(1, len(secciones) + 1)):
        errores.append(f'{ruta}: las secciones numeradas deben ir 1, 2, 3… en orden; '
                       f'hay {secciones}')
    return errores


def contar_pruebas(raiz, objetivo):
    """Pruebas (`def test_`) de un paquete (todas las de su carpeta test/) o de un archivo."""
    if objetivo.endswith('.py'):
        rutas = [os.path.join(c, objetivo) for c, _, a in os.walk(os.path.join(raiz, 'ros2_ws'))
                 if objetivo in a]
    else:
        carpeta = os.path.join(raiz, 'ros2_ws', 'src', objetivo, 'test')
        rutas = [os.path.join(carpeta, a) for a in os.listdir(carpeta)] \
            if os.path.isdir(carpeta) else []
    if not rutas:
        return None
    return sum(len(re.findall(r'^\s*def test_', open(r, encoding='utf-8').read(), flags=re.M))
               for r in rutas if r.endswith('.py'))


def revisar_pruebas(raiz, ruta, texto):
    errores = []
    for n, total, objetivo in re.findall(r'\*\*(\d+) de (\d+)\*\* en `([^`]+)`', texto):
        real = contar_pruebas(raiz, objetivo)
        if n != total:
            errores.append(f'{ruta}: «{n} de {total}» en `{objetivo}`: no pasan todas')
        elif real is None:
            errores.append(f'{ruta}: no encuentro las pruebas de `{objetivo}`')
        elif real != int(n):
            errores.append(f'{ruta}: dice {n} pruebas en `{objetivo}` y el código tiene {real}')
    return errores


def revisar(raiz):
    errores, cache = [], {}
    for ruta in sorted(archivos_md(raiz)):
        texto = open(ruta, encoding='utf-8').read()
        relativa = os.path.relpath(ruta, raiz)
        errores += revisar_enlaces(ruta, texto, cache)
        errores += [e.replace(ruta, relativa, 1) for e in revisar_pruebas(raiz, ruta, texto)]
        if '\\operatorname' in formulas(texto):
            errores.append(f'{relativa}: usa \\operatorname (GitHub no lo muestra); usa \\mathrm')
        if os.path.basename(os.path.dirname(ruta)) == 'informe':
            errores += [e.replace(ruta, relativa, 1) for e in revisar_informe(ruta, texto)]
    return [e.replace(raiz + os.sep, '') for e in errores]
