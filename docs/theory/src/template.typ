// The project function defines how your document looks.
// It takes your content and some metadata and formats it.
// Go ahead and customize it to your liking!
#let project(title: "", subtitle: "", authors: (), date: none, body) = {
  // Set the document's basic properties.
  set document(author: authors.map(a => a.name), title: title)
  set page(numbering: "1", number-align: center)
  set text(font: "New Computer Modern", lang: "es")
  show math.equation: set text(weight: 400)
  set heading(numbering: "1.1.")

  // Set run-in subheadings, starting at level 3.
  show heading: it => {
    if it.level > 2 {
      parbreak()
      text(11pt, style: "italic", weight: "regular", it.body + ".")
    } else {
      it
    }
  }


  // Title row.
  align(center)[
    #block(text(weight: 700,1.75em, title))
    #v(0.7em, weak: true)
    #block(text(weight: 400, style: "italic", 1.25em, subtitle))
    #v(2em, weak: true)
    #date
  ]

  // Author information.
  pad(
    top: 0.5em,
    bottom: 0.5em,
    x: 2em,
    grid(
      columns: (1fr, 1fr),
      gutter: 1em,
      ..authors.map(author => align(
        center,
        [
          *#author.name* \
          #link("mailto:" + author.mail, text(size: 0.75em, font: "Cascadia Mono", author.mail))
        ]
      )),
    ),
  )

  // UNLP
  pad(
    bottom: 0.5em,
    x: 2em,
    align(
      center,
      [Facultad de Ingeniería, Universidad Nacional de La Plata]
    )
  )

  // Main body.
  set par(justify: true)

  // Underline links
  show link: underline

  body
}
