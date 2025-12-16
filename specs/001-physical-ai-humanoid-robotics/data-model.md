# Data Model: Physical AI & Humanoid Robotics

## Core Entities

### Book Chapter
- **id**: string (UUID)
- **title**: string
- **slug**: string (URL-friendly)
- **chapter_number**: integer (1-12)
- **content**: markdown string
- **learning_objectives**: array of strings
- **code_examples**: array of objects (language, code, description)
- **diagrams**: array of objects (title, description, file_path)
- **exercises**: array of objects (question, type, difficulty)
- **created_at**: datetime
- **updated_at**: datetime
- **status**: enum (draft, review, published)

### RAG Chatbot
- **id**: string (UUID)
- **session_id**: string
- **user_query**: string
- **retrieved_context**: array of strings (from book content)
- **response**: string
- **timestamp**: datetime
- **is_hallucinated**: boolean (flag for validation)
- **source_chapters**: array of strings (referenced chapter slugs)

### User
- **id**: string (UUID)
- **email**: string (unique)
- **name**: string
- **role**: enum (student, developer, educator, admin)
- **created_at**: datetime
- **last_access**: datetime

### Content Validation Log
- **id**: string (UUID)
- **chapter_id**: string (foreign key to Book Chapter)
- **validation_type**: enum (content_accuracy, technical_accuracy, scope_compliance)
- **result**: enum (pass, fail, requires_review)
- **details**: string
- **timestamp**: datetime
- **validator**: string

## Relationships

- **User** can have multiple **RAG Chatbot** sessions
- **Book Chapter** can be referenced by multiple **RAG Chatbot** responses
- **Content Validation Log** belongs to specific **Book Chapter**
- **Book Chapter** has many **exercises** and **diagrams** as embedded objects

## Validation Rules

### Book Chapter
- chapter_number must be between 1-12
- content must pass Context7 MCP grounding validation
- title and slug must be unique
- status transitions: draft → review → published only

### RAG Chatbot
- response must only contain information from book content
- is_hallucinated flag must be set based on content validation
- source_chapters must not be empty when response is generated

### Content Validation Log
- validation_type must be one of the defined enum values
- timestamp required for all entries
- details field must contain specific validation information

## State Transitions

### Book Chapter Status
- draft → review (when technical review requested)
- review → draft (when changes requested)
- review → published (when approved)

### RAG Chatbot Response Validation
- When query is processed → response generated
- When response validated → is_hallucinated flag set
- When out-of-scope query detected → response contains "Not covered in this book"