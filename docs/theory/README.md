# Material de teoría

Aquí se encuentran los archivos fuente del documento que detalla la teoría detrás del proyecto. Una copia similar fue adjunta en la entrega final de trabajo para la asignatura ["Matemática C"](https://www1.ing.unlp.edu.ar/catedras/F1304/).

[Ver el documento en formato PDF](./robot-kinematics.pdf)

Este trabajo se encuentra bajo la licencia [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International](https://creativecommons.org/licenses/by-nc-sa/4.0/).

## Compilación

El documento fue escrito en formato [Typst](https://typst.app/), similar a LaTeX pero con una sintaxis más simple y moderna. Fue elegido por sobre LaTeX porque su modo de colaboración online gratuito facilitó el trabajo en equipo.

Para compilarlo localmente, se debe instalar la herramienta [Typst CLI](https://github.com/typst/typst) y ejecutar el siguiente comando:

```bash
$ typst compile src/main.typ robot-kinematics.pdf
```

Para compilar automáticamente al guardar cambios en el archivo fuente, se puede ejecutar el siguiente comando:

```bash
$ typst watch src/main.typ robot-kinematics.pdf
```
