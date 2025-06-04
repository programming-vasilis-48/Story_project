#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LLM API Client module.

This module provides a client for making API calls to Large Language Models.
It supports OpenAI and OpenRouter APIs.
"""

import os
import requests
import json
from typing import Dict, Any, Optional, List
from dotenv import load_dotenv

class LLMClient:
    """Client for making API calls to Large Language Models."""

    def __init__(self, api_key: Optional[str] = None, provider: str = "openrouter", model: Optional[str] = None):
        """Initialize the LLM client.

        Args:
            api_key (str, optional): API key for the LLM provider. If not provided,
                                    will try to get from environment variables.
            provider (str): LLM provider name (default: "openrouter")
            model (str, optional): Model to use for text generation. If not provided,
                                  will use a default model based on the provider.
        """
        # Load environment variables
        load_dotenv()

        self.provider = provider.lower()

        # Set API key
        if api_key:
            self.api_key = api_key
        elif self.provider == "openai":
            self.api_key = os.environ.get("OPENAI_API_KEY")
        elif self.provider == "openrouter":
            self.api_key = os.environ.get("OPENROUTER_API_KEY")
        else:
            self.api_key = os.environ.get(f"{provider.upper()}_API_KEY")

        if not self.api_key:
            raise ValueError(f"API key for {provider} not provided and not found in environment variables")

        # Set model
        if model:
            self.model = model
        elif self.provider == "openai":
            self.model = os.environ.get("OPENAI_MODEL", "gpt-3.5-turbo")
        elif self.provider == "openrouter":
            self.model = os.environ.get("MODEL", "meta-llama/llama-3.1-8b-instruct:free")
        else:
            self.model = os.environ.get("MODEL", "gpt-3.5-turbo")

        # Set API endpoint
        if self.provider == "openai":
            self.api_url = "https://api.openai.com/v1/chat/completions"
        elif self.provider == "openrouter":
            self.api_url = "https://openrouter.ai/api/v1/chat/completions"
        else:
            raise ValueError(f"Unsupported provider: {provider}")

    def generate_text(self, prompt: str, system_prompt: Optional[str] = None,
                     max_tokens: int = 150, temperature: float = 0.7) -> str:
        """Generate text from a prompt.

        Args:
            prompt (str): The prompt to generate text from
            system_prompt (str, optional): System prompt to set context for the LLM
            max_tokens (int): Maximum number of tokens to generate
            temperature (float): Temperature for text generation

        Returns:
            str: Generated text
        """
        # Prepare messages
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})

        # Generate chat completion
        response = self.chat_completion(messages, max_tokens, temperature)

        # Extract and return the generated text
        if isinstance(response, dict) and "choices" in response and len(response["choices"]) > 0:
            return response["choices"][0]["message"]["content"].strip()
        else:
            return "Error: Failed to generate text"

    def chat_completion(self, messages: List[Dict[str, str]], max_tokens: int = 150,
                       temperature: float = 0.7) -> Dict[str, Any]:
        """Generate a chat completion.

        Args:
            messages (List[Dict[str, str]]): List of message dictionaries with 'role' and 'content'
            max_tokens (int): Maximum number of tokens to generate
            temperature (float): Temperature for text generation

        Returns:
            Dict[str, Any]: Response from the LLM provider
        """
        # Prepare headers
        headers = {
            "Content-Type": "application/json"
        }

        if self.provider == "openai":
            headers["Authorization"] = f"Bearer {self.api_key}"
        elif self.provider == "openrouter":
            headers["Authorization"] = f"Bearer {self.api_key}"
            headers["HTTP-Referer"] = os.environ.get("HTTP_REFERER", "https://localhost")
            headers["X-Title"] = os.environ.get("X_TITLE", "Story-Switch Game")

        # Prepare request data
        data = {
            "model": self.model,
            "messages": messages,
            "max_tokens": max_tokens,
            "temperature": temperature
        }

        try:
            # Make API request
            response = requests.post(self.api_url, headers=headers, json=data)
            response.raise_for_status()  # Raise exception for HTTP errors

            # Parse and return response
            return response.json()

        except requests.exceptions.RequestException as e:
            print(f"Error: Failed to communicate with LLM API: {str(e)}")
            return {"error": str(e)}
        except json.JSONDecodeError:
            print("Error: Invalid response from LLM API")
            return {"error": "Invalid response from LLM API"}
        except Exception as e:
            print(f"Error: {str(e)}")
            return {"error": str(e)}
