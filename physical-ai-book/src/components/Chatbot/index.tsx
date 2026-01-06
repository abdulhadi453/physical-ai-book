/**
 * Chatbot Component - Main chatbot interface
 *
 * Manages chat state, handles user input, communicates with backend API,
 * and displays conversation history with loading and error states.
 */

import React, { useState, useRef, useEffect } from 'react';
import type { ChatMessage } from './types';
import { queryBook } from '../../services/chatbotApi';
import ChatInput from './ChatInput';
import ChatMessageComponent from './ChatMessage';
import LoadingIndicator from './LoadingIndicator';
import './styles.css';

const Chatbot: React.FC = () => {
  // State management
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Ref for auto-scrolling to bottom
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Input validation (FR-009)
  const validateInput = (input: string): string | null => {
    const trimmed = input.trim();

    if (!trimmed) {
      return 'Please enter a question';
    }

    if (trimmed.length > 1000) {
      return 'Question must be 1000 characters or less';
    }

    return null;
  };

  // Handle submit (T014-T018)
  const handleSubmit = async () => {
    // Validate input (T015)
    const validationError = validateInput(inputValue);
    if (validationError) {
      setError(validationError);
      return;
    }

    // Clear any previous errors
    setError(null);

    // Create user message
    const userMessage: ChatMessage = {
      id: crypto.randomUUID(),
      type: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    // Add user message to chat
    setMessages((prev) => [...prev, userMessage]);

    // Clear input
    const question = inputValue;
    setInputValue('');

    // Set loading state (T016)
    setIsLoading(true);

    try {
      // Call backend API (T014)
      const response = await queryBook(question);

      // Handle successful response (T017)
      const assistantMessage: ChatMessage = {
        id: crypto.randomUUID(),
        type: 'assistant',
        content: response.answer,
        sources: response.sources,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      // Handle error response (T018)
      const errorMessage = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMessage);

      // Optionally add error message to chat
      const errorChatMessage: ChatMessage = {
        id: crypto.randomUUID(),
        type: 'assistant',
        content: `Sorry, I encountered an error: ${errorMessage}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorChatMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle clear chat (for User Story 3)
  const handleClear = () => {
    setMessages([]);
    setInputValue('');
    setError(null);
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Ask about the Physical AI Book</h3>
        {messages.length > 0 && (
          <button
            className="clear-button"
            onClick={handleClear}
            aria-label="Clear chat history"
          >
            Clear
          </button>
        )}
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 && (
          <div className="empty-state">
            <p>Ask any question about the Physical AI book to get started!</p>
          </div>
        )}

        {messages.map((message) => (
          <ChatMessageComponent key={message.id} message={message} />
        ))}

        {isLoading && <LoadingIndicator />}

        {/* Scroll anchor */}
        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="chatbot-error" role="alert">
          {error}
        </div>
      )}

      <ChatInput
        value={inputValue}
        onChange={setInputValue}
        onSubmit={handleSubmit}
        disabled={isLoading}
      />
    </div>
  );
};

export default Chatbot;
