---
name: chapter-author
description: >
  Generates textbook chapter MDX files for the Physical AI & Humanoid Robotics
  Docusaurus site. This skill should be used when creating new chapters, adding
  exercises, or scaffolding chapter content for the textbook. Triggers: "create
  chapter", "new chapter", "add chapter", "write chapter", "generate chapter
  content", "scaffold chapter", "add exercises to chapter". Enforces all 8
  constitution principles automatically.
---

# Chapter Author

Generate complete, constitution-compliant chapter MDX files for the Physical AI
& Humanoid Robotics textbook (Docusaurus v3).

## What This Skill Does

- Generate complete chapter MDX files with all 8 required sections
- Scaffold exercises at basic/intermediate/advanced levels
- Enforce constitution compliance via post-generation checklist
- Generate Urdu translation scaffold structure
- Produce code examples with commented/uncommented Tabs

## What This Skill Does NOT Do

- Write actual research content (use domain experts for technical accuracy)
- Create images or diagrams (generates alt-text placeholders only)
- Translate content to Urdu (generates scaffold structure only)
- Deploy or build the Docusaurus site
- Validate code execution (code must be tested separately)

## Must Avoid

- Skipping learning objectives or prerequisites sections
- Code examples without version compatibility comment on line 1
- Exercises without expected output or validation criteria
- Images without descriptive alt text
- Hardcoded API keys, secrets, or tokens in any code example
- Skipping heading levels (e.g., h2 directly to h4)
- Leaving out starter code or solution references in exercises

## Workflow

1. Determine chapter context (module, topic, position in sequence)
2. Read module reference: `references/modules.md` for module-specific objectives and content requirements
3. Generate chapter MDX using `assets/chapter-template.mdx` structure
4. Validate against `references/constitution-checklist.md`
5. Generate Urdu translation scaffold if requested

## Before Implementation

Gather context before generating:

| Source | Gather |
|--------|--------|
| **Codebase** | Scan `docs/` for existing chapters, naming conventions, sidebar order |
| **Conversation** | Review prior messages for module/topic/difficulty context |
| **Skill References** | Load `references/modules.md` for module-specific requirements |
| **Constitution** | Verify current principles in `.specify/memory/constitution.md` |

Only ask user for what cannot be determined from existing context.

## Chapter Generation

### Required Clarifications

1. **Module**: Which module (1-4, intro, setup, capstone, appendix)?
2. **Topic**: What specific topic within the module?

### Optional Clarifications (infer if possible)

3. **Chapter number**: Check existing `docs/` structure for next available position. Ask only if ambiguous.
4. **Difficulty focus**: Default to balanced mix (basic/intermediate/advanced) unless user specifies.

### Output Path

Write to: `docs/module-{N}/{chapter-slug}.mdx`

Special chapters: `docs/intro.mdx`, `docs/setup.mdx`, `docs/capstone.mdx`, `docs/appendices/{topic}.mdx`

### MDX Structure

Read `assets/chapter-template.mdx` for the exact template. Every chapter MUST include these sections in order:

1. **Frontmatter** — title, sidebar_position, description, tags, estimated_time, prerequisites
2. **Learning Objectives** — 3-5 measurable objectives using Bloom's taxonomy verbs
3. **Prerequisites** — explicit list with links to prior chapters
4. **Core Content** — theory with real-world applications, diagrams, code examples
5. **Hands-On Exercises** — 3-5 exercises (basic/intermediate/advanced)
6. **Summary & Key Takeaways** — bullet list of main points
7. **Further Reading** — official docs, papers, resources
8. **Practice Problems** — 3-5 challenge problems

### Code Example Rules (Principle 1)

Every code block MUST:
- Follow PEP 8 for Python
- Include error handling
- State version compatibility in a comment on line 1

For complex examples, provide two versions:

```mdx
<Tabs>
  <TabItem value="commented" label="With Comments" default>

```python
# Compatible with: ROS 2 Humble / Python 3.10+
# Description: [what this code does]

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes

class MinimalPublisher(Node):
    """Publish messages to a topic at fixed intervals."""

    def __init__(self):
        super().__init__('minimal_publisher')  # Node name
        # Create publisher: msg type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

  </TabItem>
  <TabItem value="clean" label="Clean Version">

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

  </TabItem>
</Tabs>
```

### Exercise Format (Principles 2, 5)

Each exercise MUST include:

```mdx
### Exercise {N}: {Title}

**Difficulty**: {Basic | Intermediate | Advanced}
**Estimated Time**: {X} minutes
**Simulation / Hardware**: {Sim only | Sim + Hardware | Hardware only}

#### Objective
{One sentence: what the student will build or demonstrate}

#### Starter Code
```python
# starter-code/{filename}.py
{minimal scaffold with TODOs}
```

#### Expected Output
```
{exact terminal output or behavior description}
```

#### Validation Criteria
- [ ] {Testable criterion 1}
- [ ] {Testable criterion 2}

<details>
<summary>Solution Reference</summary>

```python
# solutions/{filename}.py
{complete working solution}
```

</details>
```

### Accessibility (Principle 4)

- Heading hierarchy: h1 (title only) -> h2 (sections) -> h3 (subsections). Never skip levels.
- Images: `![Descriptive alt text explaining the diagram content](./img/{filename}.png)`
- Admonitions for callouts:

```mdx
:::note[For Beginners]
Simplified explanation here.
:::

:::tip[Pro Tip]
Advanced insight here.
:::

:::warning[Common Mistake]
What to avoid and why.
:::

:::danger[Safety Critical]
Hardware/security warnings here.
:::
```

### Urdu Translation Scaffold

When generating Urdu scaffold, create a parallel file at `i18n/ur/docusaurus-plugin-content-docs/current/{same-path}.mdx`:

- Translate all prose to Urdu
- Keep code blocks in English
- Translate inline code comments to Urdu
- Use `:::note` admonitions in Urdu
- Keep technical terms in English with Urdu transliteration in parentheses on first use:
  `ROS 2 (آر او ایس ٹو)`, `Node (نوڈ)`, `Topic (ٹاپک)`

### Personalization Markers (Principle 7)

Include collapsible sections for different experience levels:

```mdx
<details>
<summary>New to Python? Start here</summary>
{Beginner-friendly explanation with basic examples}
</details>

<details>
<summary>Experienced with ROS 1? Key differences</summary>
{Migration notes and comparison}
</details>
```

### Security Patterns (Principle 8)

When code involves credentials, APIs, or network:

```python
# ALWAYS use environment variables — never hardcode
import os
API_KEY = os.environ.get("OPENAI_API_KEY")
if not API_KEY:
    raise EnvironmentError("OPENAI_API_KEY not set. See .env.example")
```

### Official Documentation

| Resource | URL | Use For |
|----------|-----|---------|
| Docusaurus v3 | https://docusaurus.io/docs | MDX features, i18n, admonitions, tabs |
| ROS 2 Humble | https://docs.ros.org/en/humble/ | API references, package tutorials |
| Gazebo Sim | https://gazebosim.org/docs | Simulation setup, SDF format |
| NVIDIA Isaac Sim | https://docs.omniverse.nvidia.com/isaacsim/ | Isaac workflows, USD |
| NVIDIA Isaac ROS | https://nvidia-isaac-ros.github.io/ | Perception pipelines |
| Nav2 | https://docs.nav2.org/ | Navigation stack integration |
| OpenAI Whisper | https://github.com/openai/whisper | Voice interface reference |

For patterns not covered in `references/modules.md`, fetch from these official sources.

### Post-Generation Validation

After writing the chapter, verify against `references/constitution-checklist.md`. Report any gaps.

### Troubleshooting Section (Principle 3)

For chapters involving setup or hardware, append before Summary:

```mdx
## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| {error message} | {root cause} | {fix steps} |
| {error message} | {root cause} | {fix steps} |

:::info[Hardware Compatibility]
Tested on: {hardware/software versions}.
Known issues with: {incompatible configs}.
:::
```
