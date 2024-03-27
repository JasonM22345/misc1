import os
os.environ["CUDA_LAUNCH_BLOCKING"] = "1"

import torch
import torch.nn as nn
import torch.multiprocessing as mp
from torch.utils.data import Dataset, DataLoader
from torch.nn.utils.rnn import pad_sequence
from torch.nn import TransformerEncoder, TransformerEncoderLayer
from torch.optim.lr_scheduler import CosineAnnealingLR
from transformers import GPT2Tokenizer
import datetime
import math
import random
import time

# Define the device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Uncomment the next line to balance GPU capabilities (optional)
# os.environ["CUDA_VISIBLE_DEVICES"] = "0,1"  # Adjust GPU IDs based on your system

# Define the dataset class
class TextDataset(Dataset):
    def __init__(self, folder_path, tokenizer, split, val_ratio=0.1):
        self.tokenizer = tokenizer
        self.sentences = []
        for filename in os.listdir(folder_path):
            if filename.endswith(".txt"):
                file_path = os.path.join(folder_path, filename)
                with open(file_path, "r", encoding="utf-8") as file:
                    self.sentences.extend([line.strip() for line in file.readlines()])
        # Randomly shuffle sentences
        random.shuffle(self.sentences)
        total_len = len(self.sentences)
        split_idx = int(total_len * (1 - val_ratio))

        # Split data
        if split == 'train':
            self.sentences = self.sentences[:split_idx]
        elif split == 'val':
            self.sentences = self.sentences[split_idx:]

    def __len__(self):
        return len(self.sentences)

    def __getitem__(self, idx):
        # Here, this should convert your sentence to numerical form
        sentence = self.sentences[idx]
        tokenized_sentence = self.tokenizer.encode(sentence, max_length=1024, truncation=True)
        return torch.tensor(tokenized_sentence)

def collate_fn(batch):
    length = [len(item) for item in batch]                
    b_len = max(length)                                   
    tensor = torch.zeros((len(batch), b_len), dtype=torch.long, device=device)
    for i, item in enumerate(batch):                     
        tensor[i, :length[i]] = item
    return tensor.to(device)

# Define the PositionalEncoding class
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, dropout=0.1, max_len=5000):
        super(PositionalEncoding, self).__init__()
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model).to(device)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0).transpose(0, 1)
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + self.pe[:x.size(0), :]
        return self.dropout(x)

# Define the Transformer model
class TransformerModel(nn.Module):
    def __init__(self, ntoken, ninp, nhead, nhid, nlayers, dropout=0.5):
        super(TransformerModel, self).__init__()
        self.model_type = 'Transformer'
        self.src_mask = None
        self.batch_first = True
        self.pos_encoder = PositionalEncoding(ninp, dropout)
        encoder_layers = TransformerEncoderLayer(ninp, nhead, nhid, dropout)
        self.transformer_encoder = TransformerEncoder(encoder_layers, nlayers)
        self.encoder = nn.Embedding(ntoken, ninp)
        self.ninp = ninp
        self.decoder = nn.Linear(ninp, ntoken)

    def _generate_square_subsequent_mask(self, sz):
        mask = (torch.triu(torch.ones(sz, sz)) == 1).transpose(0, 1)
        mask = mask.float().masked_fill(mask == 0, float('-inf')).masked_fill(mask == 1, float(0.0))
        return mask

    def forward(self, x):
        if self.src_mask is None or self.src_mask.size(0) != len(x):
            device = x.device
            mask = self._generate_square_subsequent_mask(len(x)).to(device)
            self.src_mask = mask

        x = self.encoder(x.long()) * math.sqrt(self.ninp)  # Convert x to Long type
        x = self.pos_encoder(x)
        output = self.transformer_encoder(x, self.src_mask)
        output = self.decoder(output)
        return output

# Define the function to pad and numericalize the batch
def numericalize_batch(batch, tokenizer, device):
    tokenized_batch = [tokenizer.encode(sentence) for sentence in batch]
    padded_batch = pad_sequence([torch.tensor(tokens).to(device) for tokens in tokenized_batch], batch_first=True, padding_value=0)
    return padded_batch.to(device)

# Define the function to print status messages with timestamps
def print_status(message):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"{timestamp} - {message}")

def evaluate(model, data_loader, criterion):
    model.eval()
    total_loss = 0.
    with torch.no_grad():
        for batch in data_loader:
            data, targets = batch[:-1].t(), batch[1:].t().contiguous()
            output = model(data)
            loss = criterion(output.view(-1, ntokens), targets.view(-1))
            total_loss += loss.item()
    return total_loss / len(data_loader)

# Define the function to train a batch
def train_batch(model, train_loader, val_loader, optimizer, criterion, num_epochs=1, accumulation_steps=4):
    for epoch in range(num_epochs):
        print("Training Started")
        model.train()
        total_loss = 0.
        total_batches = len(train_loader)
        for batch_idx, batch in enumerate(train_loader):
            if batch_idx % 1500 == 0:
                time.sleep(45)
            batch = batch.to(device)
            data, targets = batch[:-1].t(), batch[1:].t().contiguous()
            
            # Use checkpointing to reduce memory consumption
            data = torch.utils.checkpoint.checkpoint(model, data)
            
            output = model(data)
            loss = criterion(output.view(-1, ntokens), targets.view(-1))
            loss.backward()
            total_loss += loss.item()
            
            # Gradient accumulation step
            if (batch_idx + 1) % accumulation_steps == 0 or (batch_idx + 1) == total_batches:
                torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)  # Gradient clipping
                optimizer.step()  # Update model parameters
                optimizer.zero_grad()  # Clear gradients for the next accumulation steps
                
            print('| epoch {:3d} | batch {:3d}/{:3d}| train_loss {:5.2f} |'.format(epoch, batch_idx + 1, total_batches, total_loss/(batch_idx+1)))
        val_loss = evaluate(model, val_loader, criterion)
        scheduler.step() #Scheduler step
        print('| epoch {:3d} | batch {:3d}/{:3d}| train_loss {:5.2f} | val_loss {:5.2f} |'.format(epoch, batch_idx + 1, total_batches, total_loss/(batch_idx+1), val_loss))

# Define the function to generate output
def generate_output(model, seed_text, max_chars=70, min_chars=3, device=device, tokenizer=None):
    model.eval()
    with torch.no_grad():
        input_ids = torch.tensor([tokenizer.encode(seed_text)]).to(device)

        generated_ids = []
        start_time = time.time()

        while len(generated_ids) < max_chars:
            # Generate the next token
            output = model(input_ids)
            logits = output[0, -1, :]  # take the output of the last token

            # Apply softmax to get probabilities
            probabilities = torch.softmax(logits, dim=-1)

            # Set probabilities for unknown tokens to a very small value
            probabilities[tokenizer.convert_tokens_to_ids('[UNK]')] = 1e-10

            # Normalize probabilities
            probabilities /= probabilities.sum()

            # Sample the next token
            sampled_token = torch.multinomial(probabilities, 1).item()

            # Check for consecutive repeated tokens
            if len(generated_ids) > 0 and sampled_token == generated_ids[-1]:
                continue

            generated_ids.append(sampled_token)

            # Update input_ids for the next time step
            input_ids = torch.cat([input_ids, torch.tensor([sampled_token]).unsqueeze(0).to(device)], dim=-1)

            # Check if the generated output meets the minimum length requirement
            if len(generated_ids) >= min_chars:
                # Check if the generated output has reached the maximum length
                if len(generated_ids) >= max_chars:
                    break

            # Check if the generation time exceeds 20 seconds
            elapsed_time = time.time() - start_time
            if elapsed_time > 20:
                break

        generated_text = tokenizer.decode(generated_ids, skip_special_tokens=True)
        print_status(f"Generated Output: {generated_text}")
        return generated_text


if __name__ == '__main__':
    try:
        mp.set_start_method('spawn')

        tokenizer = GPT2Tokenizer.from_pretrained("gpt2")
        vocab_size = tokenizer.vocab_size
        train_input = "train"
        val_input = "val"

        current_directory = os.getcwd()
        folder_path = os.path.join(current_directory, "TrainingData")
        train_dataset = TextDataset(folder_path, tokenizer, train_input)
        val_dataset = TextDataset(folder_path, tokenizer, val_input)

        ntokens = tokenizer.vocab_size
        emsize = 200
        nhid = 200
        nlayers = 2
        nhead = 2
        dropout = 0.2
        T_max = 5  # Maximum number of iterations/epochs for scheduler
        eta_min = 0.00005  # Minimum learning rate

        model = TransformerModel(ntokens, emsize, nhead, nhid, nlayers, dropout).to(device)

        batch_size = 60
        num_workers = 0
        pin_memory = False
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers, pin_memory=pin_memory, collate_fn=collate_fn)
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers, pin_memory=pin_memory, collate_fn=collate_fn)

        print("Data Loaded")

        optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
        scheduler = CosineAnnealingLR(optimizer, T_max=T_max, eta_min=eta_min)
        criterion = nn.CrossEntropyLoss()

        train_batch(model, train_loader, val_loader, optimizer, criterion, num_epochs=5)
        print("Training Completed")

        final_model_path = os.path.join(current_directory, "final_model.pt")
        torch.save(model.state_dict(), final_model_path)
        print_status(f"Final model saved at {final_model_path}")

    except Exception as e:
        print(f"Error: {e}")
