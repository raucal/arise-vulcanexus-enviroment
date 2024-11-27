from ollama import Client
from rich import print
# client = Client(host="http://srv16:11434")
# first_time=True

class ollama_talker:
    def __init__(self, model, host="http://sid38:11434"):
        self.client = Client(host=host)
        self.first_time=True
        self.model = model


    def talk(self, prompt, model = 'llama3.1', stream = True):
        if self.first_time:
            output = self.client.generate(model=self.model, prompt=prompt, stream=True)
            self.first_time=False # Lo hemos ejecutado al menos una vez, pasamos a considerar contexto
            print('[magenta][SYSTEM]:[/magenta] Context activated')
        else:
            output = self.client.generate(model=model, prompt=prompt, stream=True, context=self.context)

        answer = ''
        for chunk in output:
            if chunk["done"] == True:
                #print("First Generate Complete")
                self.context = chunk["context"]  
                # print(f'{type(context)} | {len(context)} | [0]{type(context[0])}')
            #print(chunk)
            answer = answer + chunk["response"]
        try:
            print(f'[yellow]Assistant:[/yellow]\n{answer}')
        except:
            print(f':x:[red]Error on answer retrieval:[/red]\n {output}')
        return answer
print('Clase cargada')

# ollama = ollama_talker('llama3')
# print('[cyan]USER:[/cyan]')
# prompt = input('')
# ollama.talk(prompt)

# print('[cyan]USER:[/cyan]')
# prompt = input('')
# ollama_talker.talk(prompt)

# print('[cyan]USER:[/cyan]')
# prompt = input('')
# ollama_talker.talk(prompt)




# while True:
#     print('[cyan]USER:[/cyan]')
#     prompt = input('')

#     if first_time:
#         output = client.generate(model="llama3", prompt=prompt, stream=True)
#         first_time=False # Lo hemos ejecutado al menos una vez, pasamos a considerar contexto
#         print('[magenta][SYSTEM]:[/magenta] Context activated')
#     else:
#         output = client.generate(model="llama3", prompt=prompt, stream=True, context=context)

#     answer = ''
#     for chunk in output:
#         if chunk["done"] == True:
#             #print("First Generate Complete")
#             context = chunk["context"]  
#             print(f'{type(context)} | {len(context)} | [0]{type(context[0])}')
#         #print(chunk)
#         answer = answer + chunk["response"]
#     try:
#         print(f'[yellow]Assistant:[/yellow]\n{answer}')
#     except:
#         print(f':x:[red]Error on answer retrieval:[/red]\n {output}')