#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LLM Client for the Story-Switch application.

This module provides a client for interacting with the OpenRouter API
to generate text using the specified LLM model.
"""

import os
import json
import requests
from dotenv import load_dotenv

class LLMClient:
    """Client for interacting with the OpenRouter API."""
    
    def __init__(self):
        """Initialize the LLM client."""
        # Load environment variables
        load_dotenv()
        
        # Get API key and model from environment variables
        self.api_key = os.getenv("OPENROUTER_API_KEY")
        self.model = os.getenv("MODEL", "meta-llama/llama-3.3-8b-instruct:free")
        
        # Set API endpoint
        self.api_url = "https://openrouter.ai/api/v1/chat/completions"
        
        # Check if API key is available
        if not self.api_key:
            raise ValueError("OPENROUTER_API_KEY environment variable is not set")
    
    def generate_text(self, prompt, system_prompt=None, max_tokens=150, temperature=0.7):
        """
        Generate text using the OpenRouter API.
        
        Args:
            prompt (str): The user prompt to send to the LLM.
            system_prompt (str, optional): The system prompt to set context for the LLM.
            max_tokens (int, optional): Maximum number of tokens to generate.
            temperature (float, optional): Sampling temperature.
            
        Returns:
            str: The generated text.
        """
        # Prepare headers
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        # Prepare messages
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})
        
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
            
            # Parse response
            result = response.json()
            
            # Extract generated text
            if "choices" in result and len(result["choices"]) > 0:
                return result["choices"][0]["message"]["content"].strip()
            else:
                return "Error: No response from LLM"
        
        except requests.exceptions.RequestException as e:
            return f"Error: Failed to communicate with LLM API: {str(e)}"
        except json.JSONDecodeError:
            return "Error: Invalid response from LLM API"
        except Exception as e:
            return f"Error: {str(e)}"

# Example usage
if __name__ == "__main__":
    client = LLMClient()
    response = client.generate_text(
        prompt="Tell me a short story about a robot and a human becoming friends.",
        system_prompt="You are a creative storyteller."
    )
    print(response)
