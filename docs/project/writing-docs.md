# Writing documentation

This page states how we write the documentation of Durchblick and of every node repository. Read it before you
change a README or a page in `docs/`.

## The reader

The reader is an engineer who knows what a tensor, a buffer and VRAM are. The reader may not know how CUDA shares
memory between processes or why a GPU executes work asynchronously. Explain such things where the reader first needs
them.

## Where a page belongs

Each page has one purpose. A page that mixes two purposes becomes long and hard to scan, so split it.

| Directory | Purpose | The reader wants to | Form |
| --- | --- | --- | --- |
| `README.md` | entry | decide in two minutes whether and how to use this | short sections, each ends with one link |
| `docs/getting-started/` | tutorial | get a first result | numbered steps that always work |
| `docs/concepts/` | explanation | understand why Durchblick works as it does | prose, one diagram, one analogy at most |
| `docs/guides/` | how-to | solve one task | steps and code for that task |
| `docs/reference/` | lookup | find one option, topic or field | tables, complete, no narrative |
| `docs/integrations/` | other systems | use Durchblick with ROS 2 or another system | as a guide |
| `docs/project/` | contributors | build, test, release, compare | as a guide |
| `AGENTS.md` | coding agents | change the code safely | commands, rules, traps, telegraphic |

Durchblick is an independent system. Pages outside `docs/integrations/` do not need another framework. They name a
robot or a lab host only in an example that is marked as an example. A node for one specific device may name that
device.

## Sentences

1. **Write complete sentences.** Every sentence has a subject that acts and a plain verb. Do not drop words that the
   reader must supply.
   - Avoid: "No JPEG encoded, no ring written, nothing sent without a subscriber to exactly that."
   - Prefer: "A publisher encodes a JPEG only while a subscriber asks for JPEG."
2. **Use the active voice.** Name who does the work: the publisher, the reader, the node, you.
   - Avoid: "The rays are computed once per calibration."
   - Prefer: "The node computes the rays once per calibration."
3. **Put one idea in one sentence.** Aim for 20 words in instructions and 25 words in explanations. Use at most one
   subordinate clause.
4. **Do not hide a claim in punctuation.** A semicolon, a dash or a parenthesis that carries a second claim is a
   second sentence. Write that sentence. Parentheses are for a unit, an example value or a name in code.
5. **Give a contrast its own sentence, or delete it.** Do not append "not B", "instead of B" or "rather than B" to a
   statement about A.
   - Avoid: "Readers get a descriptor, not the pixels."
   - Prefer: "Readers get a descriptor, which states where the frame is."
6. **Explain each term at its first use, then keep it.** If a ring is a "ring", it is never a "buffer" or a "queue"
   later. The fixed terms are in the [glossary](../reference/glossary.md).
7. **Start with the content.** Do not announce what a paragraph will do. Do not summarise what the reader just read.
8. **Address the reader as "you".** Write instructions in the imperative: "Name the stream."
9. **State measured facts plainly.** Give the number, the unit and what was measured. Put the setup and the date in
   the table caption or in the detailed page, once.
10. **Use an analogy only where it shortens the explanation.** One analogy per concept, in one or two sentences, in
    the text and never in a heading. Then return to the real terms.
11. **No em dashes, no rhetorical questions, no exclamation marks, no praise of our own work.**

## Structure

- A README section fits on one screen. It ends with one link to the page that has the details.
- A concept page answers one "why" or "how does it work" question. Its title names the thing: "Rings", "Frames on
  the GPU". It states the problem first, then the mechanism, then what follows for the reader's code.
- A procedure is a numbered list with one action per step.
- Put numbers that the reader compares into a table.
- Put history into `CHANGELOG.md`. A page describes the current version. Do not write "since 0.3.30" or "before
  0.3.6" in a page, unless a reader on a supported older version needs it.
- Put open work and untested states into one section "Status and limits" at the end of the page.
- Every page starts with its title and one or two sentences that say what the reader finds there.

## A node repository

Every node repository has the same files, so that a reader who knows one node finds his way in every other.

```
README.md            entry, sections in the order below
AGENTS.md            for coding agents
CLAUDE.md            the single line "@AGENTS.md"
docs/performance.md  measurements with setup, what we changed, what did not help
docs/design.md       how the node works inside, decisions and their reasons
docs/<topic>.md      further pages when a topic needs more than a README section
```

README sections, in this order. Leave out a section that has no content.

1. **Title and two sentences.** What the node computes, from which inputs, at which rate.
2. **What it publishes.** A table: topic, message type, content.
3. **Requirements.** GPU, VRAM, Durchblick version, model licences.
4. **Install.** The commands, in order.
5. **Run.** The shortest command that produces a result, then the options that most users change.
6. **Why it is fast.** Before and after in one sentence, the three or four changes in a table, one link to
   `docs/performance.md`.
7. **Limits.** What the node cannot do and what we have not tested.
8. **Documentation.** Links to the pages in `docs/` and to Durchblick's concept pages.
9. **Licence and credits.**

## AGENTS.md

`AGENTS.md` is the file that coding agents of every vendor read. `CLAUDE.md` imports it, so that Claude Code reads
the same text. An agent can also read the README, so `AGENTS.md` does not repeat it. It contains, in this order:

1. one paragraph on what the repository is and where the human documentation is,
2. the commands to build, test and run, copy-ready, with the expected duration,
3. the rules of the code base, as imperatives,
4. the traps, each with the symptom and the fix,
5. the layout as a table of paths,
6. the defaults an agent needs most often (topics, default options, environment variables).

Write it tersely. Fragments are acceptable here, because the reader is a model that needs facts per token.

## Check before you commit

- Every relative link resolves. Every code block was run, or it is marked as untested.
- No fact from the old text is lost. No fact is new unless you verified it in the code.
- No sentence has more than 30 words.
- Search for `;`, ` (` and `not ` and check each hit against rules 4 and 5.
