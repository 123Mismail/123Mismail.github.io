# Self-Review Checklist for Chapter <NUMBER>

**Chapter Title:** <Title>
**Author:** [Your Name]
**Date:** YYYY-MM-DD

## Instructions

Complete this checklist before submitting the chapter for approval. All items must be checked (✓) to pass quality gates.

---

## Checklist Items

### 1. Structure and Completeness
- [ ] **All required sections present**: Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Mathematical Foundations, Summary, Review Questions

### 2. Code Quality
- [ ] **All code blocks have language tags**: Every fenced code block uses syntax highlighting (e.g., ```python, ```bash, ```xml)
- [ ] **Code examples are executable**: All code can be run without placeholders (tested locally or syntax-verified via context7)

### 3. Mathematical Rigor
- [ ] **Math symbols/variables defined**: Every variable in equations has explicit definition with units
- [ ] **LaTeX syntax valid**: All equations render correctly in Docusaurus (test with `npm run build`)
- [ ] **Equation labels present**: Display equations use `\tag{N.M}` for cross-referencing

### 4. Content Quality
- [ ] **No placeholder text**: No TODO, TKTK, ???, [FILL IN], or similar markers remain in the document

### 5. Technical Accuracy
- [ ] **context7 invoked for all API references**: Every code example, API call, or framework-specific syntax was verified using `mcp__context7__get-library-docs` before inclusion
- [ ] **Framework versions documented in code comments**: All code examples include version information (e.g., "# Tested with ROS 2 Humble")

### 6. Terminology and Style
- [ ] **Key terms bolded on first use**: Technical terms are **bolded** the first time they appear in the chapter
- [ ] **Terminology consistent with previous chapters**: Cross-check terms against glossary-terms-temp.md to ensure consistent usage

### 7. Visual Content (if applicable)
- [ ] **Figures/diagrams include alt text and captions**: All images use `![Alt text](path)` format with descriptive captions (e.g., "**Figure N.M:** Description")
- [ ] **Vector formats used where possible**: Diagrams are SVG or PDF (not PNG/JPG unless photos)

### 8. Pedagogical Quality
- [ ] **Review questions (≥3) test comprehension**: Questions map to Learning Outcomes and require understanding of concepts, not just memorization

### 9. Scope and Length
- [ ] **Word count 3000-6000**: Chapter length is within target range (use `wc -w <filename>` or word processor count)
- [ ] **No content beyond chapter scope**: All material directly relates to the chapter's Learning Outcomes (no tangents, no premature advanced topics)

---

## Final Sign-Off

**Checklist Completed By:** [Your Name]
**Date:** YYYY-MM-DD
**All Items Checked:** [ ] YES / [ ] NO (if NO, do not submit for approval)

**Additional Notes:**
[Any exceptions, clarifications, or items requiring reviewer attention]

---

## Submission

Once all checklist items are complete:
1. Save this checklist as `.templates/self-review-c<NUMBER>-checklist.md`
2. Update glossary-terms-temp.md with new terms from this chapter
3. Commit chapter to feature branch: `git commit -m "chap(c<NUMBER>): add <chapter-title>"`
4. Submit for approval with message: "Chapter <NUMBER> (<Title>) submitted for review. Please reply APPROVED to proceed or REQUEST_REVISION with feedback."
