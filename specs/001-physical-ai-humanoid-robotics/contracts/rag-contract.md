# RAG Contract: Retrieval-Augmented Generation for Physical AI Book

## Purpose
Define the contract for the Retrieval-Augmented Generation (RAG) system that powers the book's chatbot, ensuring zero hallucinations and strict adherence to book content only.

## RAG System Requirements

### 1. Source Content Boundaries
- **Allowed Sources**: Only content from the 12-book chapters
- **Prohibited Sources**: External websites, general knowledge, other books
- **Scope**: Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac, Unity robotics
- **Temporal Boundaries**: Information relevant to 2023-2033 timeframe

### 2. Response Generation Rules
- **Book-Only Answers**: Responses must derive exclusively from book content
- **Zero Hallucination Tolerance**: No invented facts, APIs, or workflows
- **Chapter Awareness**: Responses should indicate which chapters provide information
- **Confidence Scoring**: Provide confidence level for each response

## Technical Architecture

### 1. Vector Database (Qdrant Cloud)
- **Embedding Model**: OpenAI text-embedding-ada-002 or equivalent
- **Chunk Size**: 512-1024 tokens per chunk
- **Overlap**: 10% overlap between chunks to preserve context
- **Metadata**: Store chapter number, section, and content type with each chunk

### 2. Retrieval Process
- **Query Embedding**: Convert user query to vector representation
- **Similarity Search**: Find top-k most similar content chunks
- **Relevance Scoring**: Score retrieved chunks by relevance to query
- **Context Assembly**: Combine relevant chunks into context for generation

### 3. Generation Process
- **Model**: OpenAI GPT-4 or equivalent
- **Context Window**: Respect token limits while maximizing relevant information
- **Prompt Engineering**: Use book-specific instructions to prevent hallucination
- **Output Validation**: Verify response content against source material

## Response Behavior Rules

### 1. In-Scope Queries
When a query can be answered from book content:
- Provide accurate answer based on book chapters
- Reference specific chapters that contain the information
- Include relevant code examples or concepts from the book
- Maintain educational tone appropriate for target audience

### 2. Out-of-Scope Queries
When a query cannot be answered from book content:
- **Response**: "Not covered in this book"
- **No Invention**: Never create information that isn't in the book
- **Scope Clarification**: Briefly explain the book's scope if helpful
- **No External References**: Don't suggest looking elsewhere

### 3. Partial Information Queries
When only partial information exists in the book:
- Provide the available information from book content
- Clearly indicate what is known vs. unknown
- Reference the specific chapters with relevant information
- Avoid filling gaps with assumptions

## Validation Mechanisms

### 1. Hallucination Detection
- **Fact Checking**: Compare responses against source chapters
- **API Validation**: Verify any mentioned APIs against book content
- **Parameter Verification**: Confirm parameters match book examples
- **Workflow Validation**: Ensure described processes exist in book

### 2. Source Attribution
- **Chapter References**: List chapters that support the response
- **Content Tracing**: Trace each claim back to specific book content
- **Evidence Links**: Provide specific section references when possible
- **Confidence Indicators**: Show confidence based on source strength

### 3. Scope Compliance
- **Topic Verification**: Ensure response stays within book scope
- **Temporal Checking**: Verify information is within 2023-2033 timeframe
- **Technical Boundary**: Confirm technical details match book content
- **Audience Appropriateness**: Maintain beginner-friendly but technically rigorous tone

## MCP Integration for RAG

### 1. Pre-Response MCP Validation
- Query MCP for technical accuracy before generating response
- Verify any technical claims against MCP documentation
- Cross-reference book content with MCP when available
- Flag potential discrepancies for review

### 2. MCP Fallback for RAG
When MCP contradicts book content:
- Prioritize MCP documentation for technical accuracy
- Note the discrepancy in the response
- Suggest the book content may need updating
- Maintain "Not covered in this book" when MCP has no relevant information

## Performance Requirements

### 1. Response Time
- **P95 Latency**: < 200ms for RAG response
- **Query Processing**: < 100ms for retrieval phase
- **Generation Time**: < 100ms for response generation
- **MCP Integration**: Account for additional MCP query time

### 2. Accuracy Metrics
- **Hallucination Rate**: 0% (absolute requirement)
- **Accuracy Rate**: > 95% factually correct responses
- **Relevance Score**: > 90% relevant responses to queries
- **Source Attribution**: 100% of claims linked to book content

### 3. Availability
- **Uptime**: 99.9% availability for RAG service
- **MCP Integration**: Graceful degradation when MCP unavailable
- **Fallback Handling**: Proper responses when content is missing
- **Error Recovery**: Automatic recovery from transient failures

## Security and Privacy

### 1. Data Handling
- **Query Privacy**: User queries not stored permanently
- **Session Management**: Secure session handling
- **Content Protection**: Book content protected from extraction
- **Access Control**: Proper authentication and authorization

### 2. Content Security
- **No Content Leakage**: Prevent extraction of book content through queries
- **Rate Limiting**: Prevent content scraping via high-volume queries
- **Query Analysis**: Monitor for attempts to bypass scope restrictions
- **Response Sanitization**: Ensure responses don't contain sensitive information

## Error Handling

### 1. Common Error Scenarios
- **No Relevant Content**: "Not covered in this book"
- **MCP Unavailable**: Respond from book content only
- **System Overload**: Graceful degradation with reduced functionality
- **Invalid Query**: Politely ask for clarification within book scope

### 2. Fallback Behaviors
- **Primary**: Respond from book content with proper attribution
- **Secondary**: "Not covered in this book" when no relevant content
- **Tertiary**: Suggest related topics within book scope if helpful
- **Never**: Generate content from external sources or internal knowledge

## Monitoring and Observability

### 1. Key Metrics
- **Hallucination Detection Rate**: Track false positive/negative rates
- **Response Accuracy**: Regular evaluation of response correctness
- **User Satisfaction**: Feedback on response quality and relevance
- **System Performance**: Latency, availability, and throughput metrics

### 2. Logging Requirements
- **Query-Response Pairs**: For quality assurance and debugging
- **Source Attribution**: Track which book content was used
- **Error Cases**: Log all error scenarios for analysis
- **MCP Interactions**: Monitor MCP query success/failure rates

## Compliance Verification

### 1. Hallucination Testing
- Regular evaluation with known-fact questions
- Testing with technical detail queries
- Validation against MCP documentation
- Assessment of scope compliance

### 2. Content Accuracy
- Verification of technical claims against book content
- Checking of code examples and parameters
- Validation of configuration and workflow descriptions
- Assessment of educational appropriateness

This contract ensures that the RAG system for the Physical AI & Humanoid Robotics book delivers accurate, book-only responses with zero hallucinations while maintaining the educational quality and technical accuracy required by the project constitution.