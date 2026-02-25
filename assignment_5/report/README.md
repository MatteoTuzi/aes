# Compilazione del report

```bash
cd assignment_5/report
pdflatex report.tex
pdflatex report.tex   # seconda passata per indice e riferimenti
```

Oppure con latexmk:
```bash
latexmk -pdf report.tex
```

Le immagini devono essere nella cartella `assignment_5/images/`.
