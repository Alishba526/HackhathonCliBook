// Advanced Robotics Chatbot with Google AI for Hackathon
(function() {
    // Google AI API Configuration
    const GOOGLE_API_KEY = 'AIzaSyDOwi-jtA89mvy5HkJmPlPSHS26ZrcjnN4';
    const GOOGLE_API_URL = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent';

    // Create chatbot container
    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'robotics-chatbot';
    chatbotContainer.innerHTML = `
        <div id="chatbot-toggle" style="
            position: fixed;
            bottom: 20px;
            right: 20px;
            width: 60px;
            height: 60px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 50%;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 1000;
            transition: all 0.3s ease;
        ">
            <span id="chatbot-icon" style="color: white; font-size: 24px;">🤖</span>
        </div>
        <div id="chatbot-window" style="
            position: fixed;
            bottom: 90px;
            right: 20px;
            width: 350px;
            height: 500px;
            background: white;
            border-radius: 12px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
            display: none;
            flex-direction: column;
            z-index: 1000;
            overflow: hidden;
        ">
            <div id="chatbot-header" style="
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                padding: 15px;
                font-weight: bold;
                display: flex;
                justify-content: space-between;
                align-items: center;
            ">
                <span>🤖 AI Robotics Assistant</span>
                <span id="chatbot-close" style="cursor: pointer; font-size: 20px;">×</span>
            </div>
            <div id="chatbot-messages" style="
                flex: 1;
                padding: 15px;
                overflow-y: auto;
                background: #f8f9fa;
            ">
                <div class="message bot" style="
                    margin-bottom: 10px;
                    padding: 10px;
                    background: white;
                    border-radius: 8px;
                    border-left: 4px solid #667eea;
                ">
                    Hi! I'm your AI Robotics Assistant powered by Google Gemini! 🤖<br><br>
                    I can help you with:<br>
                    • ROS 2 and robotic systems<br>
                    • Gazebo & Unity simulation<br>
                    • NVIDIA Isaac & AI<br>
                    • Vision-Language-Action (VLA)<br>
                    • General robotics questions<br><br>
                    Ask me anything about robotics!
                </div>
            </div>
            <div id="chatbot-input" style="
                padding: 15px;
                border-top: 1px solid #e9ecef;
                display: flex;
                gap: 10px;
            ">
                <input type="text" id="chatbot-text" placeholder="Ask me about robotics..." style="
                    flex: 1;
                    padding: 8px 12px;
                    border: 1px solid #ced4da;
                    border-radius: 6px;
                    outline: none;
                ">
                <button id="chatbot-send" style="
                    padding: 8px 15px;
                    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                    color: white;
                    border: none;
                    border-radius: 6px;
                    cursor: pointer;
                ">Send</button>
            </div>
        </div>
    `;
    document.body.appendChild(chatbotContainer);

    // Google AI API call function
    async function getAIResponse(message) {
        try {
            const response = await fetch(`${GOOGLE_API_URL}?key=${GOOGLE_API_KEY}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    contents: [{
                        parts: [{
                            text: `You are an expert robotics AI assistant for a Physical AI & Humanoid Robotics textbook. The textbook covers:

Modules:
1. ROS 2 (Robotic Nervous System) - nodes, topics, services, URDF
2. Digital Twin (Gazebo & Unity) - simulation environments
3. AI Robot Brain (NVIDIA Isaac) - perception and planning
4. Vision-Language-Action (VLA) - voice-to-action control

Capstone: Autonomous humanoid with voice commands

Keep responses helpful, accurate, and focused on robotics. If the question is not robotics-related, gently redirect to robotics topics.

User question: ${message}`
                        }]
                    }]
                })
            });

            if (!response.ok) {
                throw new Error('API request failed');
            }

            const data = await response.json();
            return data.candidates[0].content.parts[0].text;
        } catch (error) {
            console.error('Google AI API error:', error);
            // Fallback to simple responses
            return getFallbackResponse(message);
        }
    }

    // Fallback responses for when API fails
    function getFallbackResponse(message) {
        const responses = {
            'ros': 'ROS 2 is the robotic nervous system! It manages communication between robot components using nodes, topics, and services. Check Module 1 for details.',
            'gazebo': 'Gazebo is a physics simulator for testing robots before real hardware. It uses SDF files and integrates with ROS 2. See Module 2!',
            'unity': 'Unity provides high-fidelity 3D rendering and human-robot interaction simulation. It connects to ROS 2 via TCP. Module 2 covers this.',
            'isaac': 'NVIDIA Isaac provides photorealistic simulation and hardware-accelerated perception. It includes Isaac Sim and Isaac ROS. Module 3 details!',
            'vla': 'Vision-Language-Action combines vision (CLIP), language (Whisper/GPT), and action (ROS 2) for voice-controlled robots. Module 4 explains this.',
            'capstone': 'The capstone project builds an autonomous humanoid that listens to voice commands and executes tasks in simulation. It uses all 4 modules!',
            'modules': 'There are 4 modules: 1) ROS 2 (nervous system), 2) Digital Twin (Gazebo/Unity), 3) AI Brain (Isaac), 4) VLA (voice-action).',
            'default': 'I\'m here to help with robotics questions! Ask about ROS 2, Gazebo, Unity, Isaac, or VLA systems.'
        };

        const msg = message.toLowerCase();
        if (msg.includes('ros')) return responses.ros;
        if (msg.includes('gazebo')) return responses.gazebo;
        if (msg.includes('unity')) return responses.unity;
        if (msg.includes('isaac')) return responses.isaac;
        if (msg.includes('vla') || msg.includes('vision') || msg.includes('language') || msg.includes('action')) return responses.vla;
        if (msg.includes('capstone') || msg.includes('project')) return responses.capstone;
        if (msg.includes('module')) return responses.modules;
        return responses.default;
    }

    function addMessage(text, isUser = false) {
        const messages = document.getElementById('chatbot-messages');
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${isUser ? 'user' : 'bot'}`;
        messageDiv.style.cssText = `
            margin-bottom: 10px;
            padding: 10px;
            border-radius: 8px;
            ${isUser ?
                'background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; margin-left: 50px;' :
                'background: white; border-left: 4px solid #667eea;'
            }
        `;
        messageDiv.textContent = text;
        messages.appendChild(messageDiv);
        messages.scrollTop = messages.scrollHeight;
    }

    // Show typing indicator
    function showTyping() {
        const messages = document.getElementById('chatbot-messages');
        const typingDiv = document.createElement('div');
        typingDiv.id = 'typing-indicator';
        typingDiv.style.cssText = `
            margin-bottom: 10px;
            padding: 10px;
            background: white;
            border-radius: 8px;
            border-left: 4px solid #667eea;
            font-style: italic;
            color: #666;
        `;
        typingDiv.textContent = '🤖 AI is thinking...';
        messages.appendChild(typingDiv);
        messages.scrollTop = messages.scrollHeight;
    }

    function hideTyping() {
        const typingDiv = document.getElementById('typing-indicator');
        if (typingDiv) {
            typingDiv.remove();
        }
    }

    // Event listeners
    document.getElementById('chatbot-toggle').addEventListener('click', function() {
        const window = document.getElementById('chatbot-window');
        window.style.display = window.style.display === 'none' ? 'flex' : 'none';
    });

    document.getElementById('chatbot-close').addEventListener('click', function() {
        document.getElementById('chatbot-window').style.display = 'none';
    });

    document.getElementById('chatbot-send').addEventListener('click', async function() {
        const input = document.getElementById('chatbot-text');
        const message = input.value.trim();
        if (message) {
            addMessage(message, true);
            input.value = '';
            showTyping();

            try {
                const response = await getAIResponse(message);
                hideTyping();
                addMessage(response);
            } catch (error) {
                hideTyping();
                addMessage('Sorry, I\'m having trouble connecting to my AI brain right now. Let me help with some basic robotics info instead!');
                // Fallback to simple response
                setTimeout(() => {
                    addMessage(getFallbackResponse(message));
                }, 500);
            }
        }
    });

    document.getElementById('chatbot-text').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            document.getElementById('chatbot-send').click();
        }
    });
})();