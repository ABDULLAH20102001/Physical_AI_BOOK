
# Quickstart Guide: Physical AI & Humanoid Robotics

## Prerequisites

- Python 3.10+
- Node.js 18+
- ROS 2 Humble Hawksbill (for robotics simulation)
- Docker (for local development)
- Access to Context7 MCP Server
- OpenAI API key (for RAG chatbot)

## Project Setup

### 1. Clone and Initialize
```bash
git clone <repository-url>
cd physical-ai-humanoid-robotics
```

### 2. Frontend Setup (Docusaurus)
```bash
cd frontend
npm install
```

### 3. Backend Setup (FastAPI)
```bash
cd backend
pip install -r requirements.txt
```

### 4. Environment Configuration
Create `.env` file in backend directory:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
NEON_DATABASE_URL=your_neon_postgres_url
CONTEXT7_MCP_URL=your_context7_mcp_endpoint
```

## Development Workflow

### 1. Start Frontend Development Server
```bash
cd frontend
npm start
```

### 2. Start Backend Development Server
```bash
cd backend
uvicorn src.api:app --reload
```

### 3. Generate Chapter Content (MCP-Grounded)
```bash
cd scripts
python content_generator.py --chapter 1 --mcp-ground
```

### 4. Index Content for RAG
```bash
cd scripts
python rag_indexer.py --source docs/ --target qdrant
```

## Key Commands

### Content Generation
```bash
# Generate a specific chapter with MCP grounding
python scripts/content_generator.py --chapter 1 --validate

# Validate content against MCP documentation
python scripts/validator.py --content docs/chapter1.md
```

### RAG Operations
```bash
# Test RAG response
curl -X POST http://localhost:8000/api/v1/rag/query -d '{"query": "What is Physical AI?"}'

# Check hallucination detection
python scripts/hallucination_detector.py --response "test response"
```

### Validation
```bash
# Run content accuracy validation
pytest tests/validation/

# Check scope compliance
python scripts/scope_checker.py --content docs/
```

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │    │    Backend       │    │   External      │
│  (Docusaurus)   │◄──►│   (FastAPI)      │◄──►│  Services       │
│                 │    │                  │    │                 │
│ - Book chapters │    │ - Content gen    │    │ - Context7 MCP  │
│ - Chatbot UI    │    │ - RAG service    │    │ - OpenAI APIs   │
│ - Navigation    │    │ - Validation     │    │ - Qdrant Cloud  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## First Steps for Developers

1. **Setup**: Complete the environment configuration above
2. **Explore**: Run the frontend to see the current Docusaurus structure
3. **Generate**: Create your first chapter using the content generator
4. **Validate**: Ensure your content passes MCP grounding checks
5. **Integrate**: Connect your chapter to the RAG system

## Common Issues & Solutions

### MCP Connection Issues
- Ensure Context7 MCP Server is accessible
- Check API credentials in environment variables
- Verify MCP query format matches expected schema

### RAG Response Quality
- Confirm content is properly indexed in Qdrant
- Validate that MCP grounding was applied during content generation
- Check for hallucination flags in the response

### Content Scope Compliance
- All content must align with the allowed topics in the constitution
- Verify technical accuracy through MCP documentation
- Ensure no out-of-scope topics are inadvertently included

## Next Steps

1. Review the complete 12-chapter structure in the documentation
2. Set up your development environment
3. Begin with Chapter 1: Introduction to Physical AI
4. Follow the MCP-grounded content generation workflow